#pragma once

#include "scene.h"
#include "pcg.h"

/// Compute an offset shadow ray origin on the "virtual smooth surface"
/// to fix shadow terminator artifacts on smooth-shaded triangle meshes.
/// Uses Blender's quadratic surface approximation: squared barycentric weights
/// make the offset fall off faster in triangle interiors than Hanika's linear version.
inline Vector3 compute_shadow_origin(const PathVertex &vertex,
                                     const Scene &scene) {
    const Shape &shape = scene.shapes[vertex.shape_id];
    const TriangleMesh *mesh = std::get_if<TriangleMesh>(&shape);
    if (!mesh || mesh->normals.empty()) {
        return vertex.position;
    }

    Vector3i idx = mesh->indices[vertex.primitive_id];
    Vector3 v0 = mesh->positions[idx[0]];
    Vector3 v1 = mesh->positions[idx[1]];
    Vector3 v2 = mesh->positions[idx[2]];
    Vector3 n0 = mesh->normals[idx[0]];
    Vector3 n1 = mesh->normals[idx[1]];
    Vector3 n2 = mesh->normals[idx[2]];

    Real b1 = vertex.st[0];
    Real b2 = vertex.st[1];
    Real b0 = Real(1) - b1 - b2;

    Vector3 P = vertex.position;

    // Compute heights of P above each vertex's tangent plane
    Real h0 = dot(P - v0, n0);
    Real h1 = dot(P - v1, n1);
    Real h2 = dot(P - v2, n2);

    // Quadratic approximation: squared barycentric weights
    Vector3 offset = n0 * (b0 * b0 * h0) + n1 * (b1 * b1 * h1) + n2 * (b2 * b2 * h2);
    Vector3 P_offset = P - offset;

    // Convexity check: only offset on convex regions to avoid light leaks
    if (dot(P_offset - P, vertex.shading_frame.n) > 0) {
        return P_offset;
    }
    return P;
}

/// Unidirectional path tracing
Spectrum path_tracing(const Scene &scene,
                      int x, int y, /* pixel coordinates */
                      pcg32_state &rng) {
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = init_ray_differential(w, h);

    std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
    if (!vertex_) {
        // Primary ray missed geometry: use background color if set,
        // otherwise show the envmap as usual.
        if (scene.options.use_background_color) {
            return fromRGB(scene.options.background_color);
        }
        if (has_envmap(scene)) {
            const Light &envmap = get_envmap(scene);
            return emission(envmap,
                            -ray.dir, // pointing outwards from light
                            ray_diff.spread,
                            PointAndNormal{}, // dummy parameter for envmap
                            scene);
        }
        return make_zero_spectrum();
    }
    PathVertex vertex = *vertex_;

    Spectrum radiance = make_zero_spectrum();
    // A path's contribution is
    // C(v) = W(v0, v1) * G(v0, v1) * f(v0, v1, v2) *
    //                    G(v1, v2) * f(v1, v2, v3) *
    //                  ........
    //                  * G(v_{n-1}, v_n) * L(v_{n-1}, v_n)
    // where v is the path vertices, W is the sensor response
    // G is the geometry term, f is the BSDF, L is the emission
    //
    // "sample_primary" importance samples both W and G,
    // and we assume it always has weight 1.

    // current_path_throughput stores the ratio between
    // 1) the path contribution from v0 up to v_{i} (the BSDF f(v_{i-1}, v_i, v_{i+1}) is not included),
    // where i is where the PathVertex "vertex" lies on, and
    // 2) the probability density for computing the path v from v0 up to v_i,
    // so that we can compute the Monte Carlo estimates C/p.
    Spectrum current_path_throughput = fromRGB(Vector3{1, 1, 1});
    // eta_scale stores the scale introduced by Snell-Descartes law to the BSDF (eta^2).
    // We use the same Russian roulette strategy as Mitsuba/pbrt-v3
    // and tracking eta_scale and removing it from the
    // path contribution is crucial for many bounces of refraction.
    Real eta_scale = Real(1);

    // We hit a light immediately.
    // This path has only two vertices and has contribution
    // C = W(v0, v1) * G(v0, v1) * L(v0, v1)
    if (is_light(scene.shapes[vertex.shape_id])) {
        radiance += current_path_throughput *
            emission(vertex, -ray.dir, scene);
    }

    // We iteratively sum up path contributions from paths with different number of vertices
    // If max_depth == -1, we rely on Russian roulette for path termination.
    int max_depth = scene.options.max_depth;
    for (int num_vertices = 3; max_depth == -1 || num_vertices <= max_depth + 1; num_vertices++) {
        // We are at v_i, and all the path contribution on and before has been accounted for.
        // Now we need to somehow generate v_{i+1} to account for paths with more vertices.
        // In path tracing, we generate two vertices:
        // 1) we sample a point on the light source (often called "Next Event Estimation")
        // 2) we randomly trace a ray from the surface point at v_i and hope we hit something.
        //
        // The first importance samples L(v_i, v_{i+1}), and the second
        // importance samples f(v_{i-1}, v_i, v_{i+1}) * G(v_i, v_{i+1})
        //
        // We then combine the two sampling strategies to estimate the contribution using weighted average.
        // Say the contribution of the first sampling is C1 (with probability density p1),
        // and the contribution of the second sampling is C2 (with probability density p2,
        // then we compute the estimate as w1*C1/p1 + w2*C2/p2.
        //
        // Assuming the vertices for C1 is v^1, and v^2 for C2,
        // Eric Veach showed that it is a good idea setting
        // w1 = p_1(v^1)^k / (p_1(v^1)^k + p_2(v^1)^k)
        // w2 = p_2(v^2)^k / (p_1(v^2)^k + p_2(v^2)^k),
        // where k is some scalar real number, and p_a(v^b) is the probability density of generating
        // vertices v^b using sampling method "a".
        // We will set k=2 as suggested by Eric Veach.

        // Finally, we set our "next vertex" in the loop to the v_{i+1} generated
        // by the second sampling, and update current_path_throughput using
        // our hemisphere sampling.

        // Let's implement this!
        const Material &mat = scene.materials[vertex.material_id];

        // First, we sample a point on the light source.
        // We do this by first picking a light source, then pick a point on it.
        Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real light_w = next_pcg32_real<Real>(rng);
        Real shape_w = next_pcg32_real<Real>(rng);
        int light_id = sample_light(scene, light_w);
        const Light &light = scene.lights[light_id];
        PointAndNormal point_on_light =
            sample_point_on_light(light, vertex.position, light_uv, shape_w, scene);

        // Next, we compute w1*C1/p1. We store C1/p1 in C1.
        Spectrum C1 = make_zero_spectrum();
        Real w1 = 0;
        // Remember "current_path_throughput" already stores all the path contribution on and before v_i.
        // So we only need to compute G(v_{i}, v_{i+1}) * f(v_{i-1}, v_{i}, v_{i+1}) * L(v_{i}, v_{i+1})
        {
            // Let's first deal with C1 = G * f * L.
            // Let's first compute G.
            Real G = 0;
            Vector3 dir_light;

            // The geometry term is different between directional light sources and
            // others. Currently we only have environment maps as directional light sources.
            if (!is_envmap(light)) {
                dir_light = normalize(point_on_light.position - vertex.position);

                // Blender-style shadow terminator geometry offset.
                // Only applies significant offset near the actual geometric terminator
                // (where NgL ~ 0), controlled by offset_cutoff parameter.
                Vector3 shadow_origin = vertex.position;
                {
                    Real NL = dot(vertex.shading_frame.n, dir_light);
                    Real NgL = dot(vertex.geometric_normal, dir_light);
                    constexpr Real offset_cutoff = Real(0.1);
                    Real offset_amount;
                    if (NL < offset_cutoff) {
                        offset_amount = std::clamp(Real(2) - (NgL + NL) / offset_cutoff, Real(0), Real(1));
                    } else {
                        offset_amount = std::clamp(Real(1) - NgL / offset_cutoff, Real(0), Real(1));
                    }
                    if (offset_amount > 0) {
                        Vector3 P_offset = compute_shadow_origin(vertex, scene);
                        shadow_origin = vertex.position + offset_amount * (P_offset - vertex.position);
                    }
                }

                // Use offset origin for occlusion test only;
                // direction, G, and distance use vertex.position (correct path-space quantities).
                Ray shadow_ray{shadow_origin, dir_light,
                               get_shadow_epsilon(scene),
                               (1 - get_shadow_epsilon(scene)) *
                                   distance(point_on_light.position, vertex.position)};
                if (!occluded(scene, shadow_ray)) {
                    G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                        distance_squared(point_on_light.position, vertex.position);
                }
            } else {
                // The direction from envmap towards the point is stored in
                // point_on_light.normal.
                dir_light = -point_on_light.normal;

                // Blender-style shadow terminator geometry offset (same as above).
                Vector3 shadow_origin = vertex.position;
                {
                    Real NL = dot(vertex.shading_frame.n, dir_light);
                    Real NgL = dot(vertex.geometric_normal, dir_light);
                    constexpr Real offset_cutoff = Real(0.1);
                    Real offset_amount;
                    if (NL < offset_cutoff) {
                        offset_amount = std::clamp(Real(2) - (NgL + NL) / offset_cutoff, Real(0), Real(1));
                    } else {
                        offset_amount = std::clamp(Real(1) - NgL / offset_cutoff, Real(0), Real(1));
                    }
                    if (offset_amount > 0) {
                        Vector3 P_offset = compute_shadow_origin(vertex, scene);
                        shadow_origin = vertex.position + offset_amount * (P_offset - vertex.position);
                    }
                }

                // Use offset origin for occlusion test only.
                Ray shadow_ray{shadow_origin, dir_light,
                               get_shadow_epsilon(scene),
                               infinity<Real>() /* envmaps are infinitely far away */};
                if (!occluded(scene, shadow_ray)) {
                    G = 1;
                }
            }

            // Before we proceed, we first compute the probability density p1(v1)
            // The probability density for light sampling to sample our point is
            // just the probability of sampling a light times the probability of sampling a point
            Real p1 = light_pmf(scene, light_id) *
                pdf_point_on_light(light, point_on_light, vertex.position, scene);

            // We don't need to continue the computation if G is 0.
            // Also sometimes there can be some numerical issue such that we generate
            // a light path with probability zero
            if (G > 0 && p1 > 0) {
                // Let's compute f (BSDF) next.
                Vector3 dir_view = -ray.dir;
                assert(vertex.material_id >= 0);
                Spectrum f = eval(mat, dir_view, dir_light, vertex, scene.texture_pool);

                // Evaluate the emission
                // We set the footprint to zero since it is not fully clear how
                // to set it in this case.
                // One way is to use a roughness based heuristics, but we have multi-layered BRDFs.
                // See "Real-time Shading with Filtered Importance Sampling" from Colbert et al.
                // for the roughness based heuristics.
                Spectrum L = emission(light, -dir_light, Real(0), point_on_light, scene);

                // C1 is just a product of all of them!
                // C1 is just a product of all of them!
                C1 = G * f * L;

                // Next let's compute w1

                // Remember that we want to set
                // w1 = p_1(v^1)^2 / (p_1(v^1)^2 + p_2(v^1)^2)
                // Notice that all of the probability density share the same path prefix and those cancel out.
                // Therefore we only need to account for the generation of the vertex v_{i+1}.

                // The probability density for our hemispherical sampling to sample
                Real p2 = pdf_sample_bsdf(
                    mat, dir_view, dir_light, vertex, scene.texture_pool);
                // !!!! IMPORTANT !!!!
                // In general, p1 and p2 now live in different spaces!!
                // our BSDF API outputs a probability density in the solid angle measure
                // while our light probability density is in the area measure.
                // We need to make sure that they are in the same space.
                // This can be done by accounting for the Jacobian of the transformation
                // between the two measures.
                // In general, I recommend to transform everything to area measure
                // (except for directional lights) since it fits to the path-space math better.
                // Converting a solid angle measure to an area measure is just a
                // multiplication of the geometry term G (let solid angle be dS, area be dA,
                // we have dA/dS = G).
                p2 *= G;

                w1 = (p1*p1) / (p1*p1 + p2*p2);
                C1 /= p1;
            }
        }
        radiance += current_path_throughput * C1 * w1;

        // Let's do the hemispherical sampling next.
        Vector3 dir_view = -ray.dir;
        Vector2 bsdf_rnd_param_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real bsdf_rnd_param_w = next_pcg32_real<Real>(rng);
        std::optional<BSDFSampleRecord> bsdf_sample_ =
            sample_bsdf(mat,
                        dir_view,
                        vertex,
                        scene.texture_pool,
                        bsdf_rnd_param_uv,
                        bsdf_rnd_param_w);
        if (!bsdf_sample_) {
            // BSDF sampling failed. Abort the loop.
            break;
        }
        const BSDFSampleRecord &bsdf_sample = *bsdf_sample_;
        Vector3 dir_bsdf = bsdf_sample.dir_out;
        // Update ray differentials & eta_scale
        if (bsdf_sample.eta == 0) {
            ray_diff.spread = reflect(ray_diff, vertex.mean_curvature, bsdf_sample.roughness);
        } else {
            ray_diff.spread = refract(ray_diff, vertex.mean_curvature, bsdf_sample.eta, bsdf_sample.roughness);
            eta_scale /= (bsdf_sample.eta * bsdf_sample.eta);
        }

        // Trace a ray towards bsdf_dir. Apply blended Hanika offset
        // to avoid self-intersection at convex edges in the terminator zone.
        /*
	Vector3 bsdf_origin = vertex.position;
        {
            Real cos_shade = dot(vertex.shading_frame.n, dir_bsdf);
            Real cos_geo = dot(vertex.geometric_normal, dir_bsdf);
            if (cos_shade > 0 && cos_geo < cos_shade) {
                Vector3 P_offset = compute_shadow_origin(vertex, scene);
                Real t = Real(1) - std::clamp(cos_geo / cos_shade, Real(0), Real(1));
                bsdf_origin = vertex.position + t * (P_offset - vertex.position);
            }
        }
        Ray bsdf_ray{bsdf_origin, dir_bsdf, get_intersection_epsilon(scene), infinity<Real>()};
        */
	Ray bsdf_ray{vertex.position, dir_bsdf, get_intersection_epsilon(scene), infinity<Real>()};
	std::optional<PathVertex> bsdf_vertex = intersect(scene, bsdf_ray);

        // To update current_path_throughput
        // we need to multiply G(v_{i}, v_{i+1}) * f(v_{i-1}, v_{i}, v_{i+1}) to it
        // and divide it with the pdf for getting v_{i+1} using hemisphere sampling.
        Real G;
        if (bsdf_vertex) {
            G = fabs(dot(dir_bsdf, bsdf_vertex->geometric_normal)) /
                distance_squared(bsdf_vertex->position, vertex.position);
        } else {
            // We hit nothing, set G to 1 to account for the environment map contribution.
            G = 1;
        }

        Spectrum f = eval(mat, dir_view, dir_bsdf, vertex, scene.texture_pool);
        Real p2 = pdf_sample_bsdf(mat, dir_view, dir_bsdf, vertex, scene.texture_pool);
        if (p2 <= 0) {
            // Numerical issue -- we generated some invalid rays.
            break;
        }

        // Remember to convert p2 to area measure!
        p2 *= G;
        // note that G cancels out in the division f/p, but we still need
        // G later for the calculation of w2.

        // Now we want to check whether dir_bsdf hit a light source, and
        // account for the light contribution (C2 & w2 & p2).
        // There are two possibilities: either we hit an emissive surface,
        // or we hit an environment map.
        // We will handle them separately.
        if (bsdf_vertex && is_light(scene.shapes[bsdf_vertex->shape_id])) {
            // G & f are already computed.
            Spectrum L = emission(*bsdf_vertex, -dir_bsdf, scene);
            Spectrum C2 = G * f * L;
            // Next let's compute p1(v2): the probability of the light source sampling
            // directly drawing the point corresponds to bsdf_dir.
            int light_id = get_area_light_id(scene.shapes[bsdf_vertex->shape_id]);
            assert(light_id >= 0);
            const Light &light = scene.lights[light_id];
            PointAndNormal light_point{bsdf_vertex->position, bsdf_vertex->geometric_normal};
            Real p1 = light_pmf(scene, light_id) *
                pdf_point_on_light(light, light_point, vertex.position, scene);
            Real w2 = (p2*p2) / (p1*p1 + p2*p2);

            C2 /= p2;
            radiance += current_path_throughput * C2 * w2;
        } else if (!bsdf_vertex && has_envmap(scene)) {
            // G & f are already computed.
            const Light &light = get_envmap(scene);
            Spectrum L = emission(light,
                                  -dir_bsdf, // pointing outwards from light
                                  ray_diff.spread,
                                  PointAndNormal{}, // dummy parameter for envmap
                                  scene);
            Spectrum C2 = G * f * L;
            // Next let's compute p1(v2): the probability of the light source sampling
            // directly drawing the direction bsdf_dir.
            PointAndNormal light_point{Vector3{0, 0, 0}, -dir_bsdf}; // pointing outwards from light
            Real p1 = light_pmf(scene, scene.envmap_light_id) *
                      pdf_point_on_light(light, light_point, vertex.position, scene);
            Real w2 = (p2*p2) / (p1*p1 + p2*p2);

            C2 /= p2;
            radiance += current_path_throughput * C2 * w2;
        }

        if (!bsdf_vertex) {
            // Hit nothing -- can't continue tracing.
            break;
        }

        // Update rays/intersection/current_path_throughput/current_pdf
        // Russian roulette heuristics
        Real rr_prob = 1;
        if (num_vertices - 1 >= scene.options.rr_depth) {
            rr_prob = min(max((1 / eta_scale) * current_path_throughput), Real(0.95));
            if (next_pcg32_real<Real>(rng) > rr_prob) {
                // Terminate the path
                break;
            }
        }

        ray = bsdf_ray;
        vertex = *bsdf_vertex;
        current_path_throughput = current_path_throughput * (G * f) / (p2 * rr_prob);
    }
    return radiance;
}
