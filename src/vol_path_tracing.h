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
  // Homework 2: implememt this!
  return make_zero_spectrum();
}

// The fifth volumetric renderer:
// multiple monochromatic homogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
Spectrum vol_path_tracing_5(const Scene &scene, int x,
                            int y, /* pixel coordinates */
                            pcg32_state &rng) {
  // Homework 2: implememt this!
  return make_zero_spectrum();
}

// The final volumetric renderer:
// multiple chromatic heterogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
Spectrum vol_path_tracing(const Scene &scene, int x,
                          int y, /* pixel coordinates */
                          pcg32_state &rng) {
  // Homework 2: implememt this!
  return make_zero_spectrum();
}
