/// NprDiffuse: DisneyDiffuse with effective color = toon * base_color + sphere

Spectrum eval_op::operator()(const NprDiffuse &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) <= 0) {
        return make_zero_spectrum();
    }
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }
    if (dot(frame.n, dir_out) <= 0) {
        return make_zero_spectrum();
    }

    Spectrum base_color = bsdf.toon * eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool)
                        + bsdf.sphere;
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real subsurface = eval(bsdf.subsurface, vertex.uv, vertex.uv_screen_size, texture_pool);

    Vector3 h = normalize(dir_in + dir_out);
    Real cos_theta_d = dot(h, dir_out);
    Real cos_theta_in = fabs(dot(frame.n, dir_in));
    Real cos_theta_out = fabs(dot(frame.n, dir_out));

    Real F_D90 = 0.5 + 2.0 * roughness * cos_theta_d * cos_theta_d;

    Real F_in_diff = 1.0 + (F_D90 - 1.0) * pow(1.0 - cos_theta_in, 5);
    Real F_out_diff = 1.0 + (F_D90 - 1.0) * pow(1.0 - cos_theta_out, 5);

    Spectrum f_baseDiffuse = base_color * (F_in_diff * F_out_diff * cos_theta_out / c_PI);

    Real F_SS90 = roughness * cos_theta_d * cos_theta_d;

    Real F_in_ss = 1.0 + (F_SS90 - 1.0) * pow(1.0 - cos_theta_in, 5);
    Real F_out_ss = 1.0 + (F_SS90 - 1.0) * pow(1.0 - cos_theta_out, 5);

    Real cos_sum = cos_theta_in + cos_theta_out;
    Real ss_denominator = (cos_sum > 0) ? (1.0 / cos_sum) : 0.0;

    Spectrum f_subsurface = (1.25 * base_color / c_PI) * (F_in_ss * F_out_ss * (ss_denominator - 0.5) + 0.5) * cos_theta_out;

    return (1.0 - subsurface) * f_baseDiffuse + subsurface * f_subsurface;
}

Real pdf_sample_bsdf_op::operator()(const NprDiffuse &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0) {
        return 0;
    }
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }
    if (dot(frame.n, dir_out) <= 0) {
        return 0;
    }

    return fmax(dot(frame.n, dir_out), Real(0)) / c_PI;
}

std::optional<BSDFSampleRecord> sample_bsdf_op::operator()(const NprDiffuse &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0) {
        return {};
    }
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }

    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    return BSDFSampleRecord{
        to_world(frame, sample_cos_hemisphere(rnd_param_uv)),
        Real(0), roughness};
}

TextureSpectrum get_texture_op::operator()(const NprDiffuse &bsdf) const {
    return bsdf.base_color;
}
