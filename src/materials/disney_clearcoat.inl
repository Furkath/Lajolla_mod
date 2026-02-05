#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneyClearcoat &bsdf) const {
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
    //Spectrum base_color = eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool);
    //Real rough = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    //Real aniso = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real gloss = eval(bsdf.clearcoat_gloss, vertex.uv, vertex.uv_screen_size, texture_pool);
    //Real alpha_min = 0.0001;
    //Real aspect = sqrt(1.0 - 0.9 * aniso);
    //Real alpha_x = fmax(alpha_min, rough * rough / aspect);
    //Real alpha_y = fmax(alpha_min, rough * rough * aspect);
    Real alpha_g = (1-gloss)*0.1+0.001*gloss;
    Real alphag2 = alpha_g*alpha_g;
    Real R_0 = 0.04;
    
    Vector3 local_in = to_local(frame, dir_in);
    Vector3 local_out= to_local(frame, dir_out);
    Vector3 h = normalize(dir_in + dir_out);
    Vector3 hl= to_local(frame, h);

    //Real F_c = schlick_fresnel(R, fmax(dot(h, dir_out),0.0001));
    Real F_c = R_0 + (1.0 - R_0) * pow(1.0 - fmax(dot(h, dir_out),0.0001), 5);
    //Real term_x = (hl.x * hl.x) / (alpha_x * alpha_x);
    //Real term_y = (hl.y * hl.y) / (alpha_y * alpha_y);
    //Real D_m = 1.0 / (c_PI * alpha_x * alpha_y * pow(term_x + term_y + hl.z * hl.z, 2));
    Real D_c = (alphag2-1)/(c_PI*log(alphag2)*(1+(alphag2-1)*hl.z*hl.z));
    
    auto lambda = [&](const Vector3 &w) {
        if (w.z <= 0) return Real(0);
        Real numerator = pow(w.x * 0.25, 2) + pow(w.y * 0.25, 2);
        return (sqrt(1.0 + numerator / (w.z * w.z)) - 1.0) / 2.0;
    };

    Real G_c = (1.0 / (1.0 + lambda(local_in))) * (1.0 / (1.0 + lambda(local_out)));

    Real ff = (F_c * D_c * G_c) / (4.0 * fmax(dot(frame.n,dir_in), 0.0001));
    return make_const_spectrum(ff);
}

Real pdf_sample_bsdf_op::operator()(const DisneyClearcoat &bsdf) const {
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
    Vector3 h = normalize(dir_in + dir_out);
    Vector3 hl = to_local(frame, h);

    Real clearcoat = eval(bsdf.clearcoat_gloss, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real alpha_g = (1.0 - clearcoat) * 0.1 + 0.001*clearcoat;

    // PDF of sampling h
    Real D = D_GTR1(hl.z, alpha_g);
    Real pdf_h = fmax(0.0, D * hl.z); // D(h) * cos(theta_h)

    // Jacobian for reflection: d_wh / d_wout = 1 / (4 * cos_theta_d)
    Real h_dot_out = fmax(dot(h, dir_out), Real(0));
    return pdf_h / (4.0 * h_dot_out);
    //return 0;
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneyClearcoat &bsdf) const {
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
    Real clearcoat = eval(bsdf.clearcoat_gloss, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real alpha_g = (1.0 - clearcoat) * 0.1 + 0.001*clearcoat;
    Vector3 local_h = sample_gtr1(alpha_g, rnd_param_uv);
    Vector3 h = to_world(frame, local_h);

    Real h_dot_in = dot(h, dir_in);
    Vector3 reflected = normalize(-dir_in + 2.0 * h_dot_in * h);

    if (dot(frame.n, reflected) <= 0) {
        return {};
    }

    // (Roughness here is for reference, typically used for ray differentials)
    return BSDFSampleRecord{
        reflected,
        Real(0) /* eta */, 
        alpha_g /* roughness */
    }; 
    //return {};
}

TextureSpectrum get_texture_op::operator()(const DisneyClearcoat &bsdf) const {
    return make_constant_spectrum_texture(make_zero_spectrum());
}
