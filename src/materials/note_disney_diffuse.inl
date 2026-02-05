Spectrum eval_op::operator()(const DisneyDiffuse &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) <= 0 ||
            dot(vertex.geometric_normal, dir_out) <= 0) {
        // No light below the surface
        return make_zero_spectrum();
    }
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }

    // Homework 1: implement this!
    // 1. Evaluate textures
    Spectrum base_color = eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real subsurface = eval(bsdf.subsurface, vertex.uv, vertex.uv_screen_size, texture_pool);

    // 2. Compute vectors and cosines
    Vector3 h = normalize(dir_in + dir_out); 
    Real cos_theta_d = dot(h, dir_out);
    Real cos_theta_in = fabs(dot(frame.n,dir_in));//fmax(dot(frame.n, dir_in), Real(0));
    Real cos_theta_out =fabs(dot(frame.n,dir_out));//fmax(dot(frame.n, dir_out), Real(0));

    // =========================================================
    // Base Diffuse Term (Disney Diffuse)
    // =========================================================
    Real F_D90 = 0.5 + 2.0 * roughness * cos_theta_d * cos_theta_d;
    
    // Schlick Fresnel approximation: (1 - cos)^5
    Real F_in_diff = 1.0 + (F_D90 - 1.0) * pow(1.0 - cos_theta_in, 5);
    Real F_out_diff = 1.0 + (F_D90 - 1.0) * pow(1.0 - cos_theta_out, 5);

    // Note: multiply by cos_theta_out for the rendering equation
    Spectrum f_baseDiffuse = base_color * (F_in_diff * F_out_diff * cos_theta_out / c_PI);

    // =========================================================
    // Subsurface Term (Lommel-Seeliger approximation)
    // =========================================================
    // Burley 2012: F_SS90 is different from F_D90
    Real F_SS90 = roughness * cos_theta_d * cos_theta_d;

    Real F_in_ss = 1.0 + (F_SS90 - 1.0) * pow(1.0 - cos_theta_in, 5);
    Real F_out_ss = 1.0 + (F_SS90 - 1.0) * pow(1.0 - cos_theta_out, 5);

    // Prevent divide by zero
    Real cos_sum = cos_theta_in + cos_theta_out;
    Real ss_denominator = (cos_sum > 0) ? (1.0 / cos_sum) : 0.0;

    // The formula: 1.25 * base / pi * (F_ss_in * F_ss_out * (1/(cos_in+cos_out) - 0.5) + 0.5) * cos_out
    Spectrum f_subsurface = (1.25 * base_color / c_PI) * (F_in_ss * F_out_ss * (ss_denominator - 0.5) + 0.5) * cos_theta_out;

    // =========================================================
    // Final Blending
    // =========================================================
    return (1.0 - subsurface) * f_baseDiffuse + subsurface * f_subsurface;
    //return make_zero_spectrum();
}

Real pdf_sample_bsdf_op::operator()(const DisneyDiffuse &bsdf) const {
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
    return fmax(dot(frame.n, dir_out), Real(0)) / c_PI;
    //return Real(0);
}

std::optional<BSDFSampleRecord> sample_bsdf_op::operator()(const DisneyDiffuse &bsdf) const {
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
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    return BSDFSampleRecord{
        to_world(frame, sample_cos_hemisphere(rnd_param_uv)),
        Real(0) /* eta */,/* Real(1) */ roughness};
    //return {};
}

TextureSpectrum get_texture_op::operator()(const DisneyDiffuse &bsdf) const {
    return bsdf.base_color;
}
