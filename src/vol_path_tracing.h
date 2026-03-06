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
      // transmittance is 0 if inf distance unless sigma_a 0
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
        // other kinds of hit (surfaces or lights), consider later
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

  // MIS
  Real dir_pdf = 0;
  Vector3 nee_p_cache = Vector3{0, 0, 0};
  Real multi_trans_pdf = 1;
  bool never_scatter = true;

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
              // nee hit some other things maybe obj surfaces or other lights
              blocked = true;
              break;
            }
            // index-matching surface
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

      // phase sampling
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

        // update cache for later if hiting a light
        dir_pdf = phase_pdf_val;
        nee_p_cache = ray.org;
        multi_trans_pdf = 1.0;
      } else {
        break;
      }

    } else {
      if (vertex) {
        if (vertex->material_id == -1) {
          medium_id = update_medium(*vertex, ray, medium_id);
          bounces++;
          multi_trans_pdf *= trans_pdf[0];
          ray.org =
              vertex->position + ray.dir * get_intersection_epsilon(scene);
          continue;
        }

        if (is_light(scene.shapes[vertex->shape_id])) {
          Spectrum Le = emission(*vertex, -ray.dir, scene);
          if (never_scatter) {
            // if direct hit lights, no need to do NEE
            L += throughput * Le;
          } else {
            // MIS weight
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
      }
      // hit other surfaces (objects), consider later
      break;
    }

    // Random stop
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
  // Real eta_scale = 1.0;

  // mis
  Real dir_pdf = 0;
  Vector3 nee_p_cache = Vector3{0, 0, 0};
  Real multi_trans_pdf = 1;
  bool never_scatter = true;

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

      // phase
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
      // hit
      if (vertex) {
        // index-matching
        if (vertex->material_id == -1) {
          medium_id = update_medium(*vertex, ray, medium_id);
          bounces++;
          multi_trans_pdf *= trans_pdf[0];
          ray.org =
              vertex->position + ray.dir * get_intersection_epsilon(scene);
          continue;
        }

        // light
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

        // Surface MIS (NEE + BSDF)
        const Material &mat = scene.materials[vertex->material_id];
        never_scatter = false;
        // NEE
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
          // offset a little bit to prrvent self hit.
          Vector3 shadow_p =
              vertex->position + dir_to_light * get_shadow_epsilon(scene);
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
            Real pdf_bsdf = pdf_bsdf_sa * G_light * p_trans_dir;
            Real w =
                (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_bsdf * pdf_bsdf);
            Spectrum L_emit = emission(light, -dir_to_light, ray_diff.spread,
                                       point_on_light, scene);
            L_nee = throughput * f * T_light * G_light * L_emit * w / pdf_nee;
          }
        }
        L += L_nee;

        // bsdf sampling
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
          // if (bsdf_sample->eta == 0) {
          //   ray = Ray{vertex->position, next_dir,
          //             get_intersection_epsilon(scene), infinity<Real>()};
          // } else {
          //  ray=f(eta_scale);
          // }
          //
          ray = Ray{vertex->position, next_dir, get_intersection_epsilon(scene),
                    infinity<Real>()};

          medium_id = update_medium(*vertex, ray, medium_id);

          throughput *= f / pdf_bsdf_val;

          // if (bsdf_sample->eta != 0) {
          //   eta_scale /= (bsdf_sample->eta * bsdf_sample->eta);
          // }

          dir_pdf = pdf_bsdf_val;
          nee_p_cache = vertex->position;
          multi_trans_pdf = 1.0;
        } else {
          break;
        }

      } else {
        // hit non-vertex
        // maybe envmap, skip now
        break;
      }
    }

    // random stop
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
  // Homework 2: implememt this!
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

  // same mis
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

      // now chromatic, chose a channel
      Real u_channel = next_pcg32_real<Real>(rng);
      int channel = std::clamp(int(u_channel * 3), 0, 2);
      Real accum_t = 0;
      int iteration = 0;

      // delta tracking
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
          accum_t += t;
          ray.org = ray.org + t * ray.dir;

          Spectrum sigma_a = get_sigma_a(medium, ray.org);
          Spectrum sigma_s = get_sigma_s(medium, ray.org);
          Spectrum sigma_t = sigma_a + sigma_s;
          Spectrum sigma_n = majorant - sigma_t;

          // Spectrum real_prob = sigma_t / majorant;
          // Safe per-channel division to avoid 0/0 = NaN when a channel's majorant is 0
          Spectrum real_prob;
          for (int i = 0; i < 3; i++) {
            real_prob[i] = majorant[i] > 0 ? sigma_t[i] / majorant[i] : Real(0);
          }

          if (next_pcg32_real<Real>(rng) < real_prob[channel]) {
            scatter = true;
            transmittance *= exp(-majorant * t) / max(majorant);
            trans_dir_pdf *=
                exp(-majorant * t) * majorant * real_prob / max(majorant);
            // # don’t need to account for trans_nee_pdf since we scatter
            break;
          } else {
            // all below trans_nee_pdf is better named as trans_multi_pdf, it is
            // not finally used in the part of P_nee, it is actually the same as
            // mult_trans_pdf in previous functions 4 and 5, and used for p_dir.
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
          transmittance *= exp(-majorant * dt);
          trans_dir_pdf *= exp(-majorant * dt);
          trans_nee_pdf *= exp(-majorant * dt);
          break;
        }
        iteration++;
      }
    }

    // avg trans_dir_pdf
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

      // nee with ratio tracking
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

          // ratio tracking
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
                // null hit
                Vector3 p = shadow_p + shadow_dir * accum_t;
                Spectrum sigma_a = get_sigma_a(shadow_medium, p);
                Spectrum sigma_s = get_sigma_s(shadow_medium, p);
                Spectrum sigma_t = sigma_a + sigma_s;
                Spectrum sigma_n = majorant - sigma_t;
                //Spectrum real_prob = sigma_t / majorant;
                Spectrum real_prob;
                for (int i = 0; i < 3; i++) {
                  real_prob[i] = majorant[i] > 0 ? sigma_t[i] / majorant[i] : Real(0);
                }

                T_light *= exp(-majorant * t) * sigma_n / max(majorant);
                p_trans_nee *= exp(-majorant * t) * majorant / max(majorant);
                p_trans_dir *= exp(-majorant * t) * majorant *
                               (make_const_spectrum(1) - real_prob) /
                               max(majorant);

                if (max(T_light) <= 0) {
                  break;
                }
              } else {
                // light source term
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

      // phase sampling
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
      if (vertex) {
        // index matching
        if (vertex->material_id == -1) {
          medium_id = update_medium(*vertex, ray, medium_id);
          bounces++;
          multi_trans_pdf *= trans_nee_pdf;
          ray.org =
              vertex->position + ray.dir * get_intersection_epsilon(scene);
          continue;
        }

        // light emission
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

        // surface MIS (NEE + BSDF) - also with ratio tracking
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

            // ratio tracking
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
                  //Spectrum real_prob = sigma_t / majorant;
                  Spectrum real_prob;
                  for (int i = 0; i < 3; i++) {
                    real_prob[i] = majorant[i] > 0 ? sigma_t[i] / majorant[i] : Real(0);
                  }

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

        // bsdf sampling
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
        // hit non-vertex , maybe envmap, etc
        break;
      }
    }

    // russian roulette
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
