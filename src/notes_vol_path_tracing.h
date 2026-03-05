#pragma once

#include "intersection.h"
#include "pcg.h"
#include "scene.h"

// The simplest volumetric renderer:
// single absorption only homogeneous volume
// only handle directly visible light sources
inline Spectrum vol_path_tracing_1(const Scene &scene, int x,
                                   int y, /* pixel coordinates */
                                   pcg32_state &rng) {
  int w = scene.camera.width;
  int h = scene.camera.height;
  Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                     (y + next_pcg32_real<Real>(rng)) / h);
  Ray ray = sample_primary(scene.camera, screen_pos);
  // RayDifferential ray_diff = init_ray_differential(w, h);
  RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

  std::optional<PathVertex> vertex = intersect(scene, ray, ray_diff);

  Real dist = vertex ? distance(ray.org, vertex->position) : infinity<Real>();

  Spectrum transmittance = make_const_spectrum(1.0);
  int medium_id = scene.camera.medium_id;
  if (medium_id != -1) {
    const Medium &medium = scene.media[medium_id];
    Spectrum sigma_a = get_sigma_a(medium, ray.org);

    if (dist == infinity<Real>()) {
      // If infinite distance, transmittance is 0 unless sigma_a is 0
      for (int i = 0; i < 3; ++i) {
        if (sigma_a[i] > 0)
          transmittance[i] = 0;
      }
    } else {
      transmittance = exp(-sigma_a * dist);
    }
  }

  Spectrum L = make_zero_spectrum();
  if (vertex) {
    if (is_light(scene.shapes[vertex->shape_id])) {
      L = emission(*vertex, -ray.dir, scene);
    }
  } else {
    if (has_envmap(scene)) {
      const Light &envmap = get_envmap(scene);
      L = emission(envmap, -ray.dir, ray_diff.spread, PointAndNormal{}, scene);
    }
  }

  return transmittance * L;
}

// The second simplest volumetric renderer:
// single monochromatic homogeneous volume with single scattering,
// no need to handle surface lighting, only directly visible light source
Spectrum vol_path_tracing_2(const Scene &scene, int x,
                            int y, /* pixel coordinates */
                            pcg32_state &rng) {
  int w = scene.camera.width;
  int h = scene.camera.height;
  Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                     (y + next_pcg32_real<Real>(rng)) / h);
  Ray ray = sample_primary(scene.camera, screen_pos);
  RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

  std::optional<PathVertex> vertex = intersect(scene, ray, ray_diff);
  Real t_hit = vertex ? distance(ray.org, vertex->position) : infinity<Real>();

  int medium_id = scene.camera.medium_id;

  if (medium_id == -1) {
    return make_zero_spectrum();
  }
  const Medium &medium = scene.media[medium_id];
  Spectrum sigma_a = get_sigma_a(medium, ray.org);
  Spectrum sigma_s = get_sigma_s(medium, ray.org);
  Spectrum sigma_t = sigma_a + sigma_s;

  // monochromatic
  Real st = sigma_t[0];
  Real u = next_pcg32_real<Real>(rng);
  Real t = -log(1 - u) / st;

  if (t < t_hit) {
    Vector3 p = ray.org + ray.dir * t;

    // (transmittance / trans_pdf) * sigma_s
    Spectrum throughput = sigma_s / sigma_t;

    Spectrum L_d = make_zero_spectrum();

    Real light_w = next_pcg32_real<Real>(rng);
    int light_id = sample_light(scene, light_w);
    const Light &light = scene.lights[light_id];

    Real shape_w = next_pcg32_real<Real>(rng);
    Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
    PointAndNormal point_on_light =
        sample_point_on_light(light, p, light_uv, shape_w, scene);

    Vector3 dir_to_light;
    Real dist_to_light;
    Real G = 0;

    if (is_envmap(light)) {
      dir_to_light = -point_on_light.normal;
      dist_to_light = infinity<Real>();
      Ray shadow_ray{p, dir_to_light, get_shadow_epsilon(scene), dist_to_light};
      if (!occluded(scene, shadow_ray)) {
        G = 1;
      }
    } else {
      dir_to_light = point_on_light.position - p;
      dist_to_light = length(dir_to_light);
      dir_to_light /= dist_to_light;
      Ray shadow_ray{p, dir_to_light, get_shadow_epsilon(scene),
                     dist_to_light * (1 - get_shadow_epsilon(scene))};
      if (!occluded(scene, shadow_ray)) {
        G = max(-dot(dir_to_light, point_on_light.normal), Real(0)) /
            (dist_to_light * dist_to_light);
      }
    }

    if (G > 0) {
      // Transmittance to light
      Spectrum tr_light = (dist_to_light == infinity<Real>())
                              ? make_zero_spectrum()
                              : exp(-sigma_t * dist_to_light);
      if (st <= 0 && dist_to_light == infinity<Real>())
        tr_light = make_const_spectrum(1);

      PhaseFunction phase = get_phase_function(medium);
      Spectrum phase_val = eval(phase, -ray.dir, dir_to_light);

      Spectrum L_light =
          // emission(light, -dir_to_light, Real(0), point_on_light, scene);
          emission(light, -dir_to_light, ray_diff.spread, point_on_light,
                   scene);

      Real pdf_light = light_pmf(scene, light_id) *
                       pdf_point_on_light(light, point_on_light, p, scene);

      if (pdf_light > 0) {
        L_d = throughput * L_light * G * phase_val * tr_light / pdf_light;
      }
    }

    return L_d;

  } else {
    Spectrum L = make_zero_spectrum();
    if (vertex) {
      if (is_light(scene.shapes[vertex->shape_id])) {
        L = emission(*vertex, -ray.dir, scene);
      }
    } else {
      if (has_envmap(scene)) {
        const Light &envmap = get_envmap(scene);
        L = emission(envmap, -ray.dir, ray_diff.spread, PointAndNormal{},
                     scene);
      }
    }
    return L;
  }
}

inline int update_medium(const PathVertex &vertex, const Ray &ray,
                         int medium_id) {
  if (vertex.interior_medium_id != vertex.exterior_medium_id) {
    if (dot(ray.dir, vertex.geometric_normal) > 0) {
      return vertex.exterior_medium_id;
    } else {
      return vertex.interior_medium_id;
    }
  }
  return medium_id;
}

// The third volumetric renderer (not so simple anymore):
// multiple monochromatic homogeneous volumes with multiple scattering
// no need to handle surface lighting, only directly visible light source
Spectrum vol_path_tracing_3(const Scene &scene, int x,
                            int y, /* pixel coordinates */
                            pcg32_state &rng) {
  int w = scene.camera.width;
  int h = scene.camera.height;
  Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                     (y + next_pcg32_real<Real>(rng)) / h);
  Ray ray = sample_primary(scene.camera, screen_pos);
  RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

  int medium_id = scene.camera.medium_id;
  Spectrum throughput = make_const_spectrum(1);
  Spectrum L = make_zero_spectrum();
  int bounces = 0;
  int max_depth = scene.options.max_depth;

  while (true) {
    bool scatter = false;
    std::optional<PathVertex> vertex = intersect(scene, ray, ray_diff);
    Real t_hit =
        vertex ? distance(ray.org, vertex->position) : infinity<Real>();

    Spectrum transmittance = make_const_spectrum(1);
    Spectrum trans_pdf = make_const_spectrum(1);

    if (medium_id != -1) {
      const Medium &medium = scene.media[medium_id];
      Spectrum sigma_a = get_sigma_a(medium, ray.org);
      Spectrum sigma_s = get_sigma_s(medium, ray.org);
      Spectrum sigma_t = sigma_a + sigma_s;

      // monochromatic
      Real st = sigma_t[0];
      Real u = next_pcg32_real<Real>(rng);
      Real t = -log(1 - u) / st;

      if (t < t_hit) {
        scatter = true;
        transmittance = exp(-sigma_t * t);
        trans_pdf = exp(-sigma_t * t) * sigma_t;

        ray.org += t * ray.dir;
      } else {
        transmittance = exp(-sigma_t * t_hit);
        trans_pdf = exp(-sigma_t * t_hit);
      }
    }

    throughput *= (transmittance / trans_pdf);

    if (!scatter) {
      if (!vertex) {
        if (has_envmap(scene)) {
          L += throughput * emission(get_envmap(scene), -ray.dir,
                                     ray_diff.spread, PointAndNormal{}, scene);
        }
        break;
      }

      if (is_light(scene.shapes[vertex->shape_id])) {
        L += throughput * emission(*vertex, -ray.dir, scene);
      }
    }

    if (max_depth != -1 && bounces == max_depth - 1) {
      break;
    }

    if (!scatter) {
      if (vertex && vertex->material_id == -1) {
        medium_id = update_medium(*vertex, ray, medium_id);
        bounces++;
        ray.org = vertex->position + ray.dir * get_intersection_epsilon(scene);
        continue;
      } else {
        // opaque surface and stop
        break;
      }
    }

    const Medium &medium = scene.media[medium_id];
    Spectrum sigma_s = get_sigma_s(medium, ray.org);

    PhaseFunction phase = get_phase_function(medium);
    Vector2 u{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
    std::optional<Vector3> next_dir = sample_phase_function(phase, -ray.dir, u);

    if (!next_dir)
      break;

    Spectrum phase_val = eval(phase, -ray.dir, *next_dir);
    Real phase_pdf_val = pdf_sample_phase(phase, -ray.dir, *next_dir);

    if (phase_pdf_val > 0) {
      throughput *= sigma_s * (phase_val / phase_pdf_val);
      ray.dir = *next_dir;
    } else {
      break;
    }

    if (bounces >= scene.options.rr_depth) {
      Real prob = min(max(throughput), Real(0.95));
      if (next_pcg32_real<Real>(rng) > prob) {
        break;
      }
      throughput /= prob;
    }

    bounces++;
  }

  return L;
}

// The fourth volumetric renderer:
// multiple monochromatic homogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// still no surface lighting
Spectrum vol_path_tracing_4(const Scene &scene, int x,
                            int y, /* pixel coordinates */
                            pcg32_state &rng) {
  int w = scene.camera.width;
  int h = scene.camera.height;
  Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                     (y + next_pcg32_real<Real>(rng)) / h);
  Ray ray = sample_primary(scene.camera, screen_pos);
  RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

  int medium_id = scene.camera.medium_id;
  Spectrum throughput = make_const_spectrum(1);
  Spectrum L = make_zero_spectrum();
  int bounces = 0;
  int max_depth = scene.options.max_depth;

  // Variables for MIS (following homework2.pdf Part 5)
  Real dir_pdf = 0; // PDF of sampling the last direction (solid angle)
  Vector3 nee_p_cache =
      Vector3{0, 0, 0};     // Position from which we can issue NEE
  Real multi_trans_pdf = 1; // Product PDF of transmittance sampling through
                            // index-matching surfaces
  bool never_scatter = true;

  while (true) {
    bool scatter = false;
    std::optional<PathVertex> vertex = intersect(scene, ray, ray_diff);
    Real t_hit =
        vertex ? distance(ray.org, vertex->position) : infinity<Real>();

    Spectrum transmittance = make_const_spectrum(1);
    Spectrum trans_pdf = make_const_spectrum(1);

    // Sample distance in medium
    if (medium_id != -1) {
      const Medium &medium = scene.media[medium_id];
      Spectrum sigma_a = get_sigma_a(medium, ray.org);
      Spectrum sigma_s = get_sigma_s(medium, ray.org);
      Spectrum sigma_t = sigma_a + sigma_s;

      Real st = sigma_t[0]; // monochromatic assumption for sampling
      Real u = next_pcg32_real<Real>(rng);
      Real t = -log(1 - u) / st;

      if (t < t_hit) {
        scatter = true;
        transmittance = exp(-sigma_t * t);
        trans_pdf = exp(-sigma_t * t) * sigma_t;
        ray.org += t * ray.dir;
      } else {
        transmittance = exp(-sigma_t * t_hit);
        trans_pdf = exp(-sigma_t * t_hit);
      }
    }

    throughput *= (transmittance / trans_pdf);

    if (scatter) {
      never_scatter = false;
      const Medium &medium = scene.media[medium_id];
      Spectrum sigma_s = get_sigma_s(medium, ray.org);
      PhaseFunction phase = get_phase_function(medium);

      // --- Next Event Estimation (following homework2.pdf pseudocode) ---
      Spectrum L_nee = make_zero_spectrum();

      // Sample light
      Real light_w_choice = next_pcg32_real<Real>(rng);
      int light_id = sample_light(scene, light_w_choice);
      const Light &light = scene.lights[light_id];

      // Sample point on light
      Real shape_w = next_pcg32_real<Real>(rng);
      Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
      PointAndNormal point_on_light =
          sample_point_on_light(light, ray.org, light_uv, shape_w, scene);

      Vector3 dir_to_light = point_on_light.position - ray.org;
      Real dist_to_light = length(dir_to_light);
      dir_to_light = normalize(dir_to_light);

      // Geometry term
      Real G = max(-dot(dir_to_light, point_on_light.normal), Real(0)) /
               (dist_to_light * dist_to_light);

      if (G > 0) {
        // Shadow ray loop: trace through index-matching surfaces, accumulate
        // transmittance
        Vector3 shadow_p = ray.org;
        int shadow_medium_id = medium_id;
        Spectrum T_light = make_const_spectrum(1);
        Real p_trans_dir = 1.0; // For MIS: accumulates transmittance PDF
        int shadow_bounces = 0;
        bool blocked = false;

        while (true) {
          Vector3 shadow_dir = normalize(point_on_light.position - shadow_p);
          Real next_t = distance(shadow_p, point_on_light.position);

          Ray shadow_ray{shadow_p, shadow_dir, get_shadow_epsilon(scene),
                         next_t * (1 - get_shadow_epsilon(scene))};
          RayDifferential shadow_ray_diff = RayDifferential{Real(0), Real(0)};
          std::optional<PathVertex> shadow_isect =
              intersect(scene, shadow_ray, shadow_ray_diff);

          Real isect_t = shadow_isect
                             ? distance(shadow_p, shadow_isect->position)
                             : next_t;

          // Attenuate by medium (if any)
          if (shadow_medium_id != -1) {
            const Medium &shadow_medium = scene.media[shadow_medium_id];
            Spectrum shadow_sigma_t = get_sigma_a(shadow_medium, shadow_p) +
                                      get_sigma_s(shadow_medium, shadow_p);
            Spectrum segment_trans = exp(-shadow_sigma_t * isect_t);
            T_light *= segment_trans;
            p_trans_dir *= segment_trans[0]; // Monochromatic
          }

          if (!shadow_isect) {
            // Nothing blocking, reached light
            break;
          } else {
            // Something is blocking
            if (shadow_isect->material_id >= 0) {
              // Opaque surface or different light
              if (is_light(scene.shapes[shadow_isect->shape_id]) &&
                  get_area_light_id(scene.shapes[shadow_isect->shape_id]) ==
                      light_id) {
                // Reached the target light
                break;
              }
              // Otherwise blocked
              blocked = true;
              break;
            }
            // Index-matching surface - pass through
            shadow_bounces++;
            if (max_depth != -1 && bounces + shadow_bounces + 1 >= max_depth) {
              blocked = true;
              break;
            }
            shadow_medium_id =
                update_medium(*shadow_isect, shadow_ray, shadow_medium_id);
            shadow_p = shadow_isect->position +
                       shadow_dir * get_intersection_epsilon(scene);
          }
        }

        if (!blocked && max(T_light) > 0) {
          // Compute contribution
          Spectrum phase_val = eval(phase, -ray.dir, dir_to_light);
          Real pdf_nee =
              light_pmf(scene, light_id) *
              pdf_point_on_light(light, point_on_light, ray.org, scene);

          // MIS: pdf_phase converted to area measure
          Real pdf_phase_sa = pdf_sample_phase(phase, -ray.dir, dir_to_light);
          Real pdf_phase = pdf_phase_sa * G * p_trans_dir;

          // Power heuristic
          Real w =
              (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_phase * pdf_phase);

          Spectrum L_emit = emission(light, -dir_to_light, ray_diff.spread,
                                     point_on_light, scene);

          // NEE contribution: throughput is already applied, but we need
          // sigma_s
          L_nee = throughput * sigma_s * T_light * G * phase_val * L_emit * w /
                  pdf_nee;
        }
      }
      L += L_nee;

      // --- Phase Function Sampling ---
      Vector2 u_phase{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
      std::optional<Vector3> next_dir =
          sample_phase_function(phase, -ray.dir, u_phase);
      if (!next_dir)
        break;

      Real phase_pdf_val = pdf_sample_phase(phase, -ray.dir, *next_dir);
      Spectrum phase_val = eval(phase, -ray.dir, *next_dir);

      if (phase_pdf_val > 0) {
        throughput *= sigma_s * (phase_val / phase_pdf_val);
        ray.dir = *next_dir;

        // Update cache for MIS when we hit a light
        dir_pdf = phase_pdf_val;
        nee_p_cache = ray.org; // The scattering point
        multi_trans_pdf = 1.0;
      } else {
        break;
      }

    } else {
      // Hit surface (not scattered)
      if (vertex) {
        // Index-matching handling
        if (vertex->material_id == -1) {
          medium_id = update_medium(*vertex, ray, medium_id);
          bounces++;
          // Accumulate multi_trans_pdf (transmittance PDF for this segment)
          multi_trans_pdf *= trans_pdf[0];
          ray.org =
              vertex->position + ray.dir * get_intersection_epsilon(scene);
          continue;
        }

        // Light handling
        if (is_light(scene.shapes[vertex->shape_id])) {
          Spectrum Le = emission(*vertex, -ray.dir, scene);
          if (never_scatter) {
            // Direct hit - no MIS needed
            L += throughput * Le;
          } else {
            // MIS weight for phase sampling
            int light_id = get_area_light_id(scene.shapes[vertex->shape_id]);
            const Light &hit_light = scene.lights[light_id];

            PointAndNormal light_point;
            light_point.position = vertex->position;
            light_point.normal = vertex->geometric_normal;

            Real pdf_nee =
                light_pmf(scene, light_id) *
                pdf_point_on_light(hit_light, light_point, nee_p_cache, scene);

            // Convert phase sampling PDF to area measure
            Real dist = distance(nee_p_cache, vertex->position);
            Real G = max(-dot(ray.dir, vertex->geometric_normal), Real(0)) /
                     (dist * dist);

            // Include trans_pdf for the final segment
            Real dir_pdf_area = dir_pdf * multi_trans_pdf * trans_pdf[0] * G;

            // Power heuristic
            Real w = (dir_pdf_area * dir_pdf_area) /
                     (dir_pdf_area * dir_pdf_area + pdf_nee * pdf_nee);

            L += throughput * Le * w;
          }
        }
      }
      // Reached a surface (not index-matching), stop
      break;
    }

    // Russian Roulette
    if (bounces >= scene.options.rr_depth) {
      Real prob = min(max(throughput), Real(0.95));
      if (next_pcg32_real<Real>(rng) > prob)
        break;
      throughput /= prob;
    }
    bounces++;
    if (max_depth != -1 && bounces >= max_depth)
      break;
  }

  return L;
}

// The fifth volumetric renderer:
// multiple monochromatic homogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
// The fifth volumetric renderer:
// multiple monochromatic homogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
Spectrum vol_path_tracing_5(const Scene &scene, int x,
                            int y, /* pixel coordinates */
                            pcg32_state &rng) {
  int w = scene.camera.width;
  int h = scene.camera.height;
  Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                     (y + next_pcg32_real<Real>(rng)) / h);
  Ray ray = sample_primary(scene.camera, screen_pos);
  RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

  int medium_id = scene.camera.medium_id;
  Spectrum throughput = make_const_spectrum(1);
  Spectrum L = make_zero_spectrum();
  int bounces = 0;
  int max_depth = scene.options.max_depth;
  Real eta_scale = 1.0;

  // Variables for MIS
  Real dir_pdf = 0; // PDF of sampling the last direction (solid angle)
  Vector3 nee_p_cache =
      Vector3{0, 0, 0};     // Position from which we can issue NEE
  Real multi_trans_pdf = 1; // Product PDF of transmittance sampling
  bool never_scatter = true;

  while (true) {
    bool scatter = false;
    std::optional<PathVertex> vertex = intersect(scene, ray, ray_diff);
    Real t_hit =
        vertex ? distance(ray.org, vertex->position) : infinity<Real>();

    Spectrum transmittance = make_const_spectrum(1);
    Spectrum trans_pdf = make_const_spectrum(1);

    // Sample distance in medium
    if (medium_id != -1) {
      const Medium &medium = scene.media[medium_id];
      Spectrum sigma_a = get_sigma_a(medium, ray.org);
      Spectrum sigma_s = get_sigma_s(medium, ray.org);
      Spectrum sigma_t = sigma_a + sigma_s;

      Real st = sigma_t[0];
      Real u = next_pcg32_real<Real>(rng);
      Real t = -log(1 - u) / st;

      if (t < t_hit) {
        scatter = true;
        transmittance = exp(-sigma_t * t);
        trans_pdf = exp(-sigma_t * t) * sigma_t;
        ray.org += t * ray.dir;
      } else {
        transmittance = exp(-sigma_t * t_hit);
        trans_pdf = exp(-sigma_t * t_hit);
      }
    }

    throughput *= (transmittance / trans_pdf);

    if (scatter) {
      never_scatter = false;
      const Medium &medium = scene.media[medium_id];
      Spectrum sigma_s = get_sigma_s(medium, ray.org);
      PhaseFunction phase = get_phase_function(medium);

      Spectrum L_nee = make_zero_spectrum();

      // Sample light
      Real light_w_choice = next_pcg32_real<Real>(rng);
      int light_id = sample_light(scene, light_w_choice);
      const Light &light = scene.lights[light_id];

      // Sample point on light
      Real shape_w = next_pcg32_real<Real>(rng);
      Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
      PointAndNormal point_on_light =
          sample_point_on_light(light, ray.org, light_uv, shape_w, scene);

      Vector3 dir_to_light = point_on_light.position - ray.org;
      Real dist_to_light = length(dir_to_light);
      dir_to_light = normalize(dir_to_light);

      Real G = max(-dot(dir_to_light, point_on_light.normal), Real(0)) /
               (dist_to_light * dist_to_light);

      if (G > 0) {
        // Shadow ray loop
        Vector3 shadow_p = ray.org;
        int shadow_medium_id = medium_id;
        Spectrum T_light = make_const_spectrum(1);
        Real p_trans_dir = 1.0;
        int shadow_bounces = 0;
        bool blocked = false;

        while (true) {
          Vector3 shadow_dir = normalize(point_on_light.position - shadow_p);
          Real next_t = distance(shadow_p, point_on_light.position);

          Ray shadow_ray{shadow_p, shadow_dir, get_shadow_epsilon(scene),
                         next_t * (1 - get_shadow_epsilon(scene))};
          RayDifferential shadow_ray_diff = RayDifferential{Real(0), Real(0)};
          std::optional<PathVertex> shadow_isect =
              intersect(scene, shadow_ray, shadow_ray_diff);

          Real isect_t = shadow_isect
                             ? distance(shadow_p, shadow_isect->position)
                             : next_t;

          if (shadow_medium_id != -1) {
            const Medium &shadow_medium = scene.media[shadow_medium_id];
            Spectrum shadow_sigma_t = get_sigma_a(shadow_medium, shadow_p) +
                                      get_sigma_s(shadow_medium, shadow_p);
            Spectrum segment_trans = exp(-shadow_sigma_t * isect_t);
            T_light *= segment_trans;
            p_trans_dir *= segment_trans[0];
          }

          if (!shadow_isect) {
            break;
          } else {
            if (shadow_isect->material_id >= 0) {
              if (is_light(scene.shapes[shadow_isect->shape_id]) &&
                  get_area_light_id(scene.shapes[shadow_isect->shape_id]) ==
                      light_id) {
                break;
              }
              blocked = true;
              break;
            }
            shadow_bounces++;
            if (max_depth != -1 && bounces + shadow_bounces + 1 >= max_depth) {
              blocked = true;
              break;
            }
            shadow_medium_id =
                update_medium(*shadow_isect, shadow_ray, shadow_medium_id);
            shadow_p = shadow_isect->position +
                       shadow_dir * get_intersection_epsilon(scene);
          }
        }

        if (!blocked && max(T_light) > 0) {
          Spectrum phase_val = eval(phase, -ray.dir, dir_to_light);
          Real pdf_nee =
              light_pmf(scene, light_id) *
              pdf_point_on_light(light, point_on_light, ray.org, scene);

          Real pdf_phase_sa = pdf_sample_phase(phase, -ray.dir, dir_to_light);
          Real pdf_phase = pdf_phase_sa * G * p_trans_dir;

          Real w =
              (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_phase * pdf_phase);

          Spectrum L_emit = emission(light, -dir_to_light, ray_diff.spread,
                                     point_on_light, scene);
          L_nee = throughput * sigma_s * T_light * G * phase_val * L_emit * w /
                  pdf_nee;
        }
      }
      L += L_nee;

      // --- Phase Function Sampling ---
      Vector2 u_phase{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
      std::optional<Vector3> next_dir =
          sample_phase_function(phase, -ray.dir, u_phase);
      if (!next_dir)
        break;

      Real phase_pdf_val = pdf_sample_phase(phase, -ray.dir, *next_dir);
      Spectrum phase_val = eval(phase, -ray.dir, *next_dir);

      if (phase_pdf_val > 0) {
        throughput *= sigma_s * (phase_val / phase_pdf_val);
        ray.dir = *next_dir;

        dir_pdf = phase_pdf_val;
        nee_p_cache = ray.org;
        multi_trans_pdf = 1.0;
      } else {
        break;
      }

    } else {
      // Hit surface
      if (vertex) {
        // Index-matching handling
        if (vertex->material_id == -1) {
          medium_id = update_medium(*vertex, ray, medium_id);
          bounces++;
          multi_trans_pdf *= trans_pdf[0];
          ray.org =
              vertex->position + ray.dir * get_intersection_epsilon(scene);
          continue;
        }

        // Light Emission
        if (is_light(scene.shapes[vertex->shape_id])) {
          Spectrum Le = emission(*vertex, -ray.dir, scene);
          if (never_scatter) {
            L += throughput * Le;
          } else {
            int light_id = get_area_light_id(scene.shapes[vertex->shape_id]);
            const Light &hit_light = scene.lights[light_id];

            PointAndNormal light_point;
            light_point.position = vertex->position;
            light_point.normal = vertex->geometric_normal;

            Real pdf_nee =
                light_pmf(scene, light_id) *
                pdf_point_on_light(hit_light, light_point, nee_p_cache, scene);

            Real dist = distance(nee_p_cache, vertex->position);
            Real G = max(-dot(ray.dir, vertex->geometric_normal), Real(0)) /
                     (dist * dist);

            Real dir_pdf_area = dir_pdf * multi_trans_pdf * trans_pdf[0] * G;

            Real w = (dir_pdf_area * dir_pdf_area) /
                     (dir_pdf_area * dir_pdf_area + pdf_nee * pdf_nee);

            L += throughput * Le * w;
          }
        }

        // Surface Scattering (NEE + BSDF)
        const Material &mat = scene.materials[vertex->material_id];
        never_scatter = false;

        // --- Surface NEE ---
        Spectrum L_nee = make_zero_spectrum();

        Real light_w_choice = next_pcg32_real<Real>(rng);
        int light_id = sample_light(scene, light_w_choice);
        const Light &light = scene.lights[light_id];

        Real shape_w = next_pcg32_real<Real>(rng);
        Vector2 light_uv{next_pcg32_real<Real>(rng),
                         next_pcg32_real<Real>(rng)};
        PointAndNormal point_on_light =
            sample_point_on_light(light, vertex->position, light_uv, shape_w,
                                  scene); // Using vertex->position

        Vector3 dir_to_light = point_on_light.position - vertex->position;
        Real dist_to_light = length(dir_to_light);
        dir_to_light = normalize(dir_to_light);

        Real G_light = max(-dot(dir_to_light, point_on_light.normal), Real(0)) /
                       (dist_to_light * dist_to_light);

        if (G_light > 0) {
          Vector3 shadow_p =
              vertex->position +
              dir_to_light *
                  get_shadow_epsilon(scene); // Surface NEE starts offset
          int shadow_medium_id = medium_id;
          Spectrum T_light = make_const_spectrum(1);
          Real p_trans_dir = 1.0;
          int shadow_bounces = 0;
          bool blocked = false;

          // Shadow ray loop (identical logic to volume NEE)
          while (true) {
            Vector3 shadow_dir = normalize(point_on_light.position - shadow_p);
            Real next_t = distance(shadow_p, point_on_light.position);
            Ray shadow_ray{shadow_p, shadow_dir, get_shadow_epsilon(scene),
                           next_t * (1 - get_shadow_epsilon(scene))};
            RayDifferential shadow_ray_diff = RayDifferential{Real(0), Real(0)};
            std::optional<PathVertex> shadow_isect =
                intersect(scene, shadow_ray, shadow_ray_diff);
            Real isect_t = shadow_isect
                               ? distance(shadow_p, shadow_isect->position)
                               : next_t;

            if (shadow_medium_id != -1) {
              const Medium &shadow_medium = scene.media[shadow_medium_id];
              Spectrum shadow_sigma_t = get_sigma_a(shadow_medium, shadow_p) +
                                        get_sigma_s(shadow_medium, shadow_p);
              Spectrum segment_trans = exp(-shadow_sigma_t * isect_t);
              T_light *= segment_trans;
              p_trans_dir *= segment_trans[0];
            }

            if (!shadow_isect)
              break;
            else {
              if (shadow_isect->material_id >= 0) {
                if (is_light(scene.shapes[shadow_isect->shape_id]) &&
                    get_area_light_id(scene.shapes[shadow_isect->shape_id]) ==
                        light_id)
                  break;
                blocked = true;
                break;
              }
              shadow_bounces++;
              if (max_depth != -1 &&
                  bounces + shadow_bounces + 1 >= max_depth) {
                blocked = true;
                break;
              }
              shadow_medium_id =
                  update_medium(*shadow_isect, shadow_ray, shadow_medium_id);
              shadow_p = shadow_isect->position +
                         shadow_dir * get_intersection_epsilon(scene);
            }
          }

          if (!blocked && max(T_light) > 0) {
            Spectrum f =
                eval(mat, -ray.dir, dir_to_light, *vertex, scene.texture_pool);
            Real pdf_nee = light_pmf(scene, light_id) *
                           pdf_point_on_light(light, point_on_light,
                                              vertex->position, scene);
            Real pdf_bsdf_sa = pdf_sample_bsdf(mat, -ray.dir, dir_to_light,
                                               *vertex, scene.texture_pool);
            Real pdf_bsdf = pdf_bsdf_sa * G_light *
                            p_trans_dir; // G_light here converts SA to Area
            Real w =
                (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_bsdf * pdf_bsdf);
            Spectrum L_emit = emission(light, -dir_to_light, ray_diff.spread,
                                       point_on_light, scene);
            L_nee = throughput * f * T_light * G_light * L_emit * w / pdf_nee;
          }
        }
        L += L_nee;

        // --- BSDF Sampling ---
        Vector2 bsdf_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real bsdf_w = next_pcg32_real<Real>(rng);
        std::optional<BSDFSampleRecord> bsdf_sample = sample_bsdf(
            mat, -ray.dir, *vertex, scene.texture_pool, bsdf_uv, bsdf_w);

        if (!bsdf_sample)
          break;

        Vector3 next_dir = bsdf_sample->dir_out;
        Real pdf_bsdf_val = pdf_sample_bsdf(mat, -ray.dir, next_dir, *vertex,
                                            scene.texture_pool);
        Spectrum f = eval(mat, -ray.dir, next_dir, *vertex, scene.texture_pool);

        if (pdf_bsdf_val > 0) {
          // Update Ray
          // Refraction handling for ray origin
          if (bsdf_sample->eta == 0) {
            ray = Ray{vertex->position, next_dir,
                      get_intersection_epsilon(scene), infinity<Real>()};
          } else {
            // Transmission: need to offset origin carefully?
            // intersect() handles self-intersection via epsilon, so
            // vertex->position should be fine with epsilon. update_medium is
            // handled at the END of the ray, so if we refracted INTO object,
            // medium updates when we hit the BACK side? No.
            // If we enter/exit, we need to update medium_id HERE?
            // In vol_path_tracing, typically the logic is: medium only updates
            // when hitting interface. But here we ARE at the interface. If
            // transmission happened, we assume we crossed the boundary. Usually
            // update_medium is called on intersection. But we are leaving.
            // Let's assume standard behavior: we just raytrace. If we are
            // inside, standard intersection logic handles it? Wait,
            // scene.camera.medium_id is tracked. In path_tracing.h: eta_scale
            // logic.
          }
          // For now, simple ray update
          ray = Ray{vertex->position, next_dir, get_intersection_epsilon(scene),
                    infinity<Real>()};

          // Critical Fix: Update medium if we transmit/reflect
          // update_medium uses the ray direction to decide if we are
          // entering/exiting
          medium_id = update_medium(*vertex, ray, medium_id);

          throughput *= f / pdf_bsdf_val;

          if (bsdf_sample->eta != 0) {
            eta_scale /= (bsdf_sample->eta * bsdf_sample->eta);
          }

          // Update variables for MIS
          dir_pdf = pdf_bsdf_val;
          nee_p_cache = vertex->position;
          multi_trans_pdf = 1.0;
        } else {
          break;
        }

      } else {
        // Hit Background
        // NO Envmap contribution per homework spec (assuming no envmap)
        break;
      }
    }

    // Russian Roulette (Standard)
    if (bounces >= scene.options.rr_depth) {
      Real prob = min(max(throughput), Real(0.95));
      if (next_pcg32_real<Real>(rng) > prob)
        break;
      throughput /= prob;
    }
    bounces++;
    if (max_depth != -1 && bounces >= max_depth)
      break;
  }

  return L;
}

// The final volumetric renderer:
// multiple chromatic heterogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
Spectrum vol_path_tracing(const Scene &scene, int x,
                          int y, /* pixel coordinates */
                          pcg32_state &rng) {
  int w = scene.camera.width;
  int h = scene.camera.height;
  Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                     (y + next_pcg32_real<Real>(rng)) / h);
  Ray ray = sample_primary(scene.camera, screen_pos);
  RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

  int medium_id = scene.camera.medium_id;
  Spectrum throughput = make_const_spectrum(1);
  Spectrum L = make_zero_spectrum();
  int bounces = 0;
  int max_depth = scene.options.max_depth;
  int max_null_collisions = scene.options.max_null_collisions;

  // MIS cache
  Real dir_pdf = 0;
  Vector3 nee_p_cache = Vector3{0, 0, 0};
  Spectrum multi_trans_pdf = make_const_spectrum(1);
  bool never_scatter = true;

  while (true) {
    bool scatter = false;
    std::optional<PathVertex> vertex = intersect(scene, ray, ray_diff);
    Real t_hit =
        vertex ? distance(ray.org, vertex->position) : infinity<Real>();

    Spectrum transmittance = make_const_spectrum(1);
    Spectrum trans_dir_pdf = make_const_spectrum(1);
    Spectrum trans_nee_pdf = make_const_spectrum(1);

    if (medium_id != -1) {
      const Medium &medium = scene.media[medium_id];
      Spectrum majorant = get_majorant(medium, ray);

      // Sample a channel for distance sampling
      Real u_channel = next_pcg32_real<Real>(rng);
      int channel = std::clamp(int(u_channel * 3), 0, 2);
      Real accum_t = 0;
      int iteration = 0;

      // Delta tracking loop
      while (true) {
        if (majorant[channel] <= 0) {
          break;
        }
        if (iteration >= max_null_collisions) {
          break;
        }

        Real t = -log(1 - next_pcg32_real<Real>(rng)) / majorant[channel];
        Real dt = t_hit - accum_t;

        if (t < dt) {
          // Haven't reached the surface yet
          accum_t += t;
          ray.org = ray.org + t * ray.dir;

          // Get local properties at the new position
          Spectrum sigma_a = get_sigma_a(medium, ray.org);
          Spectrum sigma_s = get_sigma_s(medium, ray.org);
          Spectrum sigma_t = sigma_a + sigma_s;
          Spectrum sigma_n = majorant - sigma_t;

          // Compute real collision probability
          Spectrum real_prob = sigma_t / majorant;

          if (next_pcg32_real<Real>(rng) < real_prob[channel]) {
            // Hit a "real" particle - scatter
            scatter = true;
            transmittance *= exp(-majorant * t) / max(majorant);
            trans_dir_pdf *=
                exp(-majorant * t) * majorant * real_prob / max(majorant);
            // Don't need to update trans_nee_pdf since we scatter
            break;
          } else {
            // Hit a "fake" particle (null collision) - continue
            transmittance *= exp(-majorant * t) * sigma_n / max(majorant);
            trans_dir_pdf *= exp(-majorant * t) * majorant *
                             (make_const_spectrum(1) - real_prob) /
                             max(majorant);
            trans_nee_pdf *= exp(-majorant * t) * majorant / max(majorant);

            if (max(transmittance) <= 0) {
              break;
            }
          }
        } else {
          // Reached the surface
          transmittance *= exp(-majorant * dt);
          trans_dir_pdf *= exp(-majorant * dt);
          trans_nee_pdf *= exp(-majorant * dt);
          break;
        }
        iteration++;
      }
    }

    // Update throughput with transmittance / avg(trans_dir_pdf)
    Real avg_trans_dir_pdf =
        (trans_dir_pdf[0] + trans_dir_pdf[1] + trans_dir_pdf[2]) / 3.0;
    if (avg_trans_dir_pdf > 0) {
      throughput *= transmittance / avg_trans_dir_pdf;
    }

    if (scatter) {
      never_scatter = false;
      const Medium &medium = scene.media[medium_id];
      Spectrum sigma_s = get_sigma_s(medium, ray.org);
      PhaseFunction phase = get_phase_function(medium);

      // Next Event Estimation with ratio tracking
      Spectrum L_nee = make_zero_spectrum();

      Real light_w_choice = next_pcg32_real<Real>(rng);
      int light_id = sample_light(scene, light_w_choice);
      const Light &light = scene.lights[light_id];

      Real shape_w = next_pcg32_real<Real>(rng);
      Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
      PointAndNormal point_on_light =
          sample_point_on_light(light, ray.org, light_uv, shape_w, scene);

      Vector3 dir_to_light = point_on_light.position - ray.org;
      Real dist_to_light = length(dir_to_light);
      dir_to_light = normalize(dir_to_light);

      Real G = max(-dot(dir_to_light, point_on_light.normal), Real(0)) /
               (dist_to_light * dist_to_light);

      if (G > 0) {
        Vector3 shadow_p = ray.org;
        int shadow_medium_id = medium_id;
        Spectrum T_light = make_const_spectrum(1);
        Spectrum p_trans_nee = make_const_spectrum(1);
        Spectrum p_trans_dir = make_const_spectrum(1);
        int shadow_bounces = 0;
        bool blocked = false;

        while (true) {
          Vector3 shadow_dir = normalize(point_on_light.position - shadow_p);
          Real next_t = distance(shadow_p, point_on_light.position);

          Ray shadow_ray{shadow_p, shadow_dir, get_shadow_epsilon(scene),
                         next_t * (1 - get_shadow_epsilon(scene))};
          RayDifferential shadow_ray_diff = RayDifferential{Real(0), Real(0)};
          std::optional<PathVertex> shadow_isect =
              intersect(scene, shadow_ray, shadow_ray_diff);

          Real isect_t = shadow_isect
                             ? distance(shadow_p, shadow_isect->position)
                             : next_t;

          // Ratio tracking for transmittance estimation
          if (shadow_medium_id != -1) {
            const Medium &shadow_medium = scene.media[shadow_medium_id];
            Spectrum majorant = get_majorant(shadow_medium, shadow_ray);

            Real u_channel = next_pcg32_real<Real>(rng);
            int channel = std::clamp(int(u_channel * 3), 0, 2);
            Real accum_t = 0;
            int iteration = 0;

            while (true) {
              if (majorant[channel] <= 0) {
                break;
              }
              if (iteration >= max_null_collisions) {
                break;
              }

              Real t = -log(1 - next_pcg32_real<Real>(rng)) / majorant[channel];
              Real dt = isect_t - accum_t;
              accum_t = min(accum_t + t, isect_t);

              if (t < dt) {
                // Null scattering event
                Vector3 p = shadow_p + shadow_dir * accum_t;
                Spectrum sigma_a = get_sigma_a(shadow_medium, p);
                Spectrum sigma_s = get_sigma_s(shadow_medium, p);
                Spectrum sigma_t = sigma_a + sigma_s;
                Spectrum sigma_n = majorant - sigma_t;
                Spectrum real_prob = sigma_t / majorant;

                T_light *= exp(-majorant * t) * sigma_n / max(majorant);
                p_trans_nee *= exp(-majorant * t) * majorant / max(majorant);
                p_trans_dir *= exp(-majorant * t) * majorant *
                               (make_const_spectrum(1) - real_prob) /
                               max(majorant);

                if (max(T_light) <= 0) {
                  break;
                }
              } else {
                // Reached the surface/intersection
                T_light *= exp(-majorant * dt);
                p_trans_nee *= exp(-majorant * dt);
                p_trans_dir *= exp(-majorant * dt);
                break;
              }
              iteration++;
            }
          }

          if (!shadow_isect) {
            break;
          } else {
            if (shadow_isect->material_id >= 0) {
              if (is_light(scene.shapes[shadow_isect->shape_id]) &&
                  get_area_light_id(scene.shapes[shadow_isect->shape_id]) ==
                      light_id) {
                break;
              }
              blocked = true;
              break;
            }
            shadow_bounces++;
            if (max_depth != -1 && bounces + shadow_bounces + 1 >= max_depth) {
              blocked = true;
              break;
            }
            shadow_medium_id =
                update_medium(*shadow_isect, shadow_ray, shadow_medium_id);
            shadow_p = shadow_isect->position +
                       shadow_dir * get_intersection_epsilon(scene);
          }
        }

        if (!blocked && max(T_light) > 0) {
          Spectrum phase_val = eval(phase, -ray.dir, dir_to_light);
          Real pdf_nee =
              light_pmf(scene, light_id) *
              pdf_point_on_light(light, point_on_light, ray.org, scene);

          Real pdf_phase_sa = pdf_sample_phase(phase, -ray.dir, dir_to_light);
          Real avg_p_trans_dir =
              (p_trans_dir[0] + p_trans_dir[1] + p_trans_dir[2]) / 3.0;
          Real pdf_phase = pdf_phase_sa * G * avg_p_trans_dir;

          Real w =
              (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_phase * pdf_phase);

          Spectrum L_emit = emission(light, -dir_to_light, ray_diff.spread,
                                     point_on_light, scene);

          Real avg_p_trans_nee =
              (p_trans_nee[0] + p_trans_nee[1] + p_trans_nee[2]) / 3.0;
          if (avg_p_trans_nee > 0) {
            L_nee = throughput * sigma_s * (T_light / avg_p_trans_nee) * G *
                    phase_val * L_emit * w / pdf_nee;
          }
        }
      }
      L += L_nee;

      // Phase function sampling
      Vector2 u_phase{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
      std::optional<Vector3> next_dir =
          sample_phase_function(phase, -ray.dir, u_phase);
      if (!next_dir)
        break;

      Real phase_pdf_val = pdf_sample_phase(phase, -ray.dir, *next_dir);
      Spectrum phase_val = eval(phase, -ray.dir, *next_dir);

      if (phase_pdf_val > 0) {
        throughput *= sigma_s * (phase_val / phase_pdf_val);
        ray.dir = *next_dir;

        dir_pdf = phase_pdf_val;
        nee_p_cache = ray.org;
        multi_trans_pdf = make_const_spectrum(1);
      } else {
        break;
      }

    } else {
      // Hit surface or empty space
      if (vertex) {
        // Index-matching surface
        if (vertex->material_id == -1) {
          medium_id = update_medium(*vertex, ray, medium_id);
          bounces++;
          multi_trans_pdf *= trans_nee_pdf;
          ray.org =
              vertex->position + ray.dir * get_intersection_epsilon(scene);
          continue;
        }

        // Light emission
        if (is_light(scene.shapes[vertex->shape_id])) {
          Spectrum Le = emission(*vertex, -ray.dir, scene);
          if (never_scatter) {
            L += throughput * Le;
          } else {
            int light_id = get_area_light_id(scene.shapes[vertex->shape_id]);
            const Light &hit_light = scene.lights[light_id];

            PointAndNormal light_point;
            light_point.position = vertex->position;
            light_point.normal = vertex->geometric_normal;

            Real pdf_nee =
                light_pmf(scene, light_id) *
                pdf_point_on_light(hit_light, light_point, nee_p_cache, scene);

            Real dist = distance(nee_p_cache, vertex->position);
            Real G = max(-dot(ray.dir, vertex->geometric_normal), Real(0)) /
                     (dist * dist);

            Spectrum full_trans_pdf = multi_trans_pdf * trans_nee_pdf;
            Real avg_full_trans_pdf =
                (full_trans_pdf[0] + full_trans_pdf[1] + full_trans_pdf[2]) /
                3.0;
            Real dir_pdf_area = dir_pdf * avg_full_trans_pdf * G;

            Real w = (dir_pdf_area * dir_pdf_area) /
                     (dir_pdf_area * dir_pdf_area + pdf_nee * pdf_nee);

            L += throughput * Le * w;
          }
        }

        // Surface MIS (NEE + BSDF) - same as vol_path_tracing_5 but with ratio
        // tracking
        const Material &mat = scene.materials[vertex->material_id];
        never_scatter = false;

        // NEE for surface
        Spectrum L_nee = make_zero_spectrum();

        Real light_w_choice = next_pcg32_real<Real>(rng);
        int light_id = sample_light(scene, light_w_choice);
        const Light &light = scene.lights[light_id];

        Real shape_w = next_pcg32_real<Real>(rng);
        Vector2 light_uv{next_pcg32_real<Real>(rng),
                         next_pcg32_real<Real>(rng)};
        PointAndNormal point_on_light = sample_point_on_light(
            light, vertex->position, light_uv, shape_w, scene);

        Vector3 dir_to_light = point_on_light.position - vertex->position;
        Real dist_to_light = length(dir_to_light);
        dir_to_light = normalize(dir_to_light);

        Real G_light = max(-dot(dir_to_light, point_on_light.normal), Real(0)) /
                       (dist_to_light * dist_to_light);

        if (G_light > 0) {
          Vector3 shadow_p =
              vertex->position + dir_to_light * get_shadow_epsilon(scene);
          int shadow_medium_id = medium_id;
          Spectrum T_light = make_const_spectrum(1);
          Spectrum p_trans_nee = make_const_spectrum(1);
          Spectrum p_trans_dir = make_const_spectrum(1);
          int shadow_bounces = 0;
          bool blocked = false;

          while (true) {
            Vector3 shadow_dir = normalize(point_on_light.position - shadow_p);
            Real next_t = distance(shadow_p, point_on_light.position);
            Ray shadow_ray{shadow_p, shadow_dir, get_shadow_epsilon(scene),
                           next_t * (1 - get_shadow_epsilon(scene))};
            RayDifferential shadow_ray_diff = RayDifferential{Real(0), Real(0)};
            std::optional<PathVertex> shadow_isect =
                intersect(scene, shadow_ray, shadow_ray_diff);
            Real isect_t = shadow_isect
                               ? distance(shadow_p, shadow_isect->position)
                               : next_t;

            // Ratio tracking for transmittance
            if (shadow_medium_id != -1) {
              const Medium &shadow_medium = scene.media[shadow_medium_id];
              Spectrum majorant = get_majorant(shadow_medium, shadow_ray);

              Real u_channel = next_pcg32_real<Real>(rng);
              int channel = std::clamp(int(u_channel * 3), 0, 2);
              Real accum_t = 0;
              int iteration = 0;

              while (true) {
                if (majorant[channel] <= 0) {
                  break;
                }
                if (iteration >= max_null_collisions) {
                  break;
                }

                Real t =
                    -log(1 - next_pcg32_real<Real>(rng)) / majorant[channel];
                Real dt = isect_t - accum_t;
                accum_t = min(accum_t + t, isect_t);

                if (t < dt) {
                  Vector3 p = shadow_p + shadow_dir * accum_t;
                  Spectrum sigma_a = get_sigma_a(shadow_medium, p);
                  Spectrum sigma_s = get_sigma_s(shadow_medium, p);
                  Spectrum sigma_t = sigma_a + sigma_s;
                  Spectrum sigma_n = majorant - sigma_t;
                  Spectrum real_prob = sigma_t / majorant;

                  T_light *= exp(-majorant * t) * sigma_n / max(majorant);
                  p_trans_nee *= exp(-majorant * t) * majorant / max(majorant);
                  p_trans_dir *= exp(-majorant * t) * majorant *
                                 (make_const_spectrum(1) - real_prob) /
                                 max(majorant);

                  if (max(T_light) <= 0) {
                    break;
                  }
                } else {
                  T_light *= exp(-majorant * dt);
                  p_trans_nee *= exp(-majorant * dt);
                  p_trans_dir *= exp(-majorant * dt);
                  break;
                }
                iteration++;
              }
            }

            if (!shadow_isect)
              break;
            else {
              if (shadow_isect->material_id >= 0) {
                if (is_light(scene.shapes[shadow_isect->shape_id]) &&
                    get_area_light_id(scene.shapes[shadow_isect->shape_id]) ==
                        light_id)
                  break;
                blocked = true;
                break;
              }
              shadow_bounces++;
              if (max_depth != -1 &&
                  bounces + shadow_bounces + 1 >= max_depth) {
                blocked = true;
                break;
              }
              shadow_medium_id =
                  update_medium(*shadow_isect, shadow_ray, shadow_medium_id);
              shadow_p = shadow_isect->position +
                         shadow_dir * get_intersection_epsilon(scene);
            }
          }

          if (!blocked && max(T_light) > 0) {
            Spectrum f =
                eval(mat, -ray.dir, dir_to_light, *vertex, scene.texture_pool);
            Real pdf_nee = light_pmf(scene, light_id) *
                           pdf_point_on_light(light, point_on_light,
                                              vertex->position, scene);
            Real pdf_bsdf_sa = pdf_sample_bsdf(mat, -ray.dir, dir_to_light,
                                               *vertex, scene.texture_pool);
            Real avg_p_trans_dir =
                (p_trans_dir[0] + p_trans_dir[1] + p_trans_dir[2]) / 3.0;
            Real pdf_bsdf = pdf_bsdf_sa * G_light * avg_p_trans_dir;
            Real w =
                (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_bsdf * pdf_bsdf);
            Spectrum L_emit = emission(light, -dir_to_light, ray_diff.spread,
                                       point_on_light, scene);
            Real avg_p_trans_nee =
                (p_trans_nee[0] + p_trans_nee[1] + p_trans_nee[2]) / 3.0;
            if (avg_p_trans_nee > 0) {
              L_nee = throughput * f * (T_light / avg_p_trans_nee) * G_light *
                      L_emit * w / pdf_nee;
            }
          }
        }
        L += L_nee;

        // BSDF sampling
        Vector2 bsdf_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real bsdf_w = next_pcg32_real<Real>(rng);
        std::optional<BSDFSampleRecord> bsdf_sample = sample_bsdf(
            mat, -ray.dir, *vertex, scene.texture_pool, bsdf_uv, bsdf_w);

        if (!bsdf_sample)
          break;

        Vector3 next_dir = bsdf_sample->dir_out;
        Real pdf_bsdf_val = pdf_sample_bsdf(mat, -ray.dir, next_dir, *vertex,
                                            scene.texture_pool);
        Spectrum f = eval(mat, -ray.dir, next_dir, *vertex, scene.texture_pool);

        if (pdf_bsdf_val > 0) {
          ray = Ray{vertex->position, next_dir, get_intersection_epsilon(scene),
                    infinity<Real>()};

          medium_id = update_medium(*vertex, ray, medium_id);

          throughput *= f / pdf_bsdf_val;

          dir_pdf = pdf_bsdf_val;
          nee_p_cache = vertex->position;
          multi_trans_pdf = make_const_spectrum(1);
        } else {
          break;
        }

      } else {
        // No intersection - break
        break;
      }
    }

    // Russian roulette
    if (bounces >= scene.options.rr_depth) {
      Real prob = min(max(throughput), Real(0.95));
      if (next_pcg32_real<Real>(rng) > prob)
        break;
      throughput /= prob;
    }
    bounces++;
    if (max_depth != -1 && bounces >= max_depth)
      break;
  }

  return L;
}
