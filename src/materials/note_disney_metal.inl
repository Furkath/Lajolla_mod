#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneyMetal &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0 ||
            dot(vertex.geometric_normal, dir_out) < 0) {
        // No light below the surface
        return make_zero_spectrum();
    }
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }
    // Homework 1: implement this!
    // 1. Evaluate Textures
    Spectrum base_color = eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real rough = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real aniso = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);

    // 2. Map parameters to alpha_x and alpha_y (Equation 9)
    Real alpha_min = 0.0001;
    Real aspect = sqrt(1.0 - 0.9 * aniso);
    Real alpha_x = fmax(alpha_min, rough * rough / aspect);
    Real alpha_y = fmax(alpha_min, rough * rough * aspect);

    // 3. Transform vectors to local shading frame
    Vector3 local_in = normalize(to_local(frame, dir_in));
    Vector3 local_out= normalize(to_local(frame, dir_out));
    Vector3 h = normalize(dir_in + dir_out);
    Vector3 hl= normalize(to_local(frame, h));
    
    // 4. Fresnel Reflection Fm (Equation 7) using your utility
    // Note: Schlick Fresnel uses dot(h, omega_out)
    Spectrum F_m = schlick_fresnel(base_color, fmax(dot(h, dir_out),0.0001));

    // 5. Anisotropic Normal Distribution Function Dm (Equation 8)
    Real term_x = (hl.x * hl.x) / (alpha_x * alpha_x);
    Real term_y = (hl.y * hl.y) / (alpha_y * alpha_y);
    Real D_m = 1.0 / (c_PI * alpha_x * alpha_y * pow(term_x + term_y + hl.z * hl.z, 2));

    // 6. Masking-Shadowing Term Gm using Smith model (Equation 10)
    auto lambda = [&](const Vector3 &w) {
        if (w.z <= 0) return Real(0);
        // Matching Equation 10: sqrt(1 + ((w.x*ax)^2 + (w.y*ay)^2) / w.z^2)
        Real numerator = pow(w.x * alpha_x, 2) + pow(w.y * alpha_y, 2);
        return (sqrt(1.0 + numerator / (w.z * w.z)) - 1.0) / 2.0;
    };
    
    Real G_m = (1.0 / (1.0 + lambda(local_in))) * (1.0 / (1.0 + lambda(local_out)));

    // 7. Final BRDF Evaluation (Equation 6)
    // The framework typically expects BSDF * cos_theta_out.
    // Eq 6: f_metal = (F*D*G) / (4 * |n.in|)
    return (F_m * D_m * G_m) / (4.0 * fmax(dot(frame.n,dir_in), 0.0001));
    //return make_zero_spectrum();
}

Real pdf_sample_bsdf_op::operator()(const DisneyMetal &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0 ||
            dot(vertex.geometric_normal, dir_out) < 0) {
        // No light below the surface
        return 0;
    }
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }
    // Homework 1: implement this!
    Vector3 half_vector = normalize(dir_in + dir_out);
    Real n_dot_in = dot(frame.n, dir_in);
    Real n_dot_out = dot(frame.n, dir_out);
    Real n_dot_h = dot(frame.n, half_vector);
    if (n_dot_out <= 0 || n_dot_h <= 0) {
        return 0;
    }

    //Spectrum S = eval(
    //    bsdf.specular_reflectance, vertex.uv, vertex.uv_screen_size, texture_pool);
    //Spectrum R = eval(
    //    bsdf.diffuse_reflectance, vertex.uv, vertex.uv_screen_size, texture_pool);
    //Real lS = luminance(S), lR = luminance(R);
    //if (lS + lR <= 0) {
    //    return 0;
    //}
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    // Clamp roughness to avoid numerical issues.
    roughness = std::clamp(roughness, Real(0.01), Real(1));
    // We use the reflectance to determine whether to choose specular sampling lobe or diffuse.
    //Real spec_prob = lS / (lS + lR);
    //Real diff_prob = 1 - spec_prob;
    // For the specular lobe, we use the ellipsoidal sampling from Heitz 2018
    // "Sampling the GGX Distribution of Visible Normals"
    // https://jcgt.org/published/0007/04/01/
    // this importance samples smith_masking(cos_theta_in) * GTR2(cos_theta_h, roughness) * cos_theta_out
    Real aniso = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real G = smith_masking_gtr2_aniso(to_local(frame, dir_in), roughness,aniso);
    Vector3 h = normalize(dir_in + dir_out);
    Vector3 hl= normalize(to_local(frame, h));
    Real D = GTR2aniso(hl, roughness, aniso);
    return (G * D) / (4.0 *fmax(n_dot_in,0.0001));
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneyMetal &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0) {
        // No light below the surface
        return {};
    }
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }
    // Homework 1: implement this!
    // Sample from the specular lobe.

        // Convert the incoming direction to local coordinates    
    Vector3 local_dir_in = to_local(frame, dir_in);
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
        // Clamp roughness to avoid numerical issues.
    roughness = std::clamp(roughness, Real(0.01), Real(1));
    Real aniso = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Vector3 local_micro_normal = sample_visible_normals_aniso(local_dir_in, roughness, aniso, rnd_param_uv);

        // Transform the micro normal to world space
    Vector3 half_vector = to_world(frame, local_micro_normal);
        // Reflect over the world space normal
    Vector3 reflected = normalize(-dir_in + 2 * dot(dir_in, half_vector) * half_vector);
    return BSDFSampleRecord{
            reflected,
            Real(0) /* eta */, roughness /* roughness */
        };
}

TextureSpectrum get_texture_op::operator()(const DisneyMetal &bsdf) const {
    return bsdf.base_color;
}
