#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneyGlass &bsdf) const {
    bool reflect = dot(vertex.geometric_normal, dir_in) *
                   dot(vertex.geometric_normal, dir_out) > 0;
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }

    // 1. Calculate Eta
    // If we are going into the surface, use eta (internal/external), else external/internal
    Real eta = dot(vertex.geometric_normal, dir_in) > 0 ? bsdf.eta : 1 / bsdf.eta;

    Spectrum base_color = eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);

    // Clamp roughness to avoid numerical issues
    roughness = std::clamp(roughness, Real(0.01), Real(1));

    // 4.Half Vector
    Vector3 half_vector;
    if (reflect) {
        half_vector = normalize(dir_in + dir_out);
    } else {
        // "Generalized half-vector" for refraction
        half_vector = normalize(dir_in + dir_out * eta);
    }

    // Flip half-vector if it's below surface
    if (dot(half_vector, frame.n) < 0) {
        half_vector = -half_vector;
    }

    // 5. Compute F / D / G
    Real h_dot_in = dot(half_vector, dir_in);
    Real F = fresnel_dielectric(h_dot_in, eta);
    
    Vector3 hl= normalize(to_local(frame, half_vector));
    //Real D = GTR2(dot(frame.n, half_vector), roughness);
    //Real G = smith_masking_gtr2(to_local(frame, dir_in), roughness) *
    //         smith_masking_gtr2(to_local(frame, dir_out), roughness);
    Real D = GTR2aniso(hl, roughness, anisotropic);
    Real G = smith_masking_gtr2_aniso(to_local(frame, dir_in), roughness,anisotropic) *
             smith_masking_gtr2_aniso(to_local(frame, dir_out),roughness,anisotropic);

    if (reflect) {
        return base_color * (F * D * G) / fmax( Real(1e-6),4 * fabs(dot(frame.n, dir_in)) );
    } else {
        Spectrum sqrt_base_color = sqrt(base_color);

        Real eta_factor = dir == TransportDirection::TO_LIGHT ? (1 / (eta * eta)) : 1;
        Real h_dot_out = dot(half_vector, dir_out);
        Real sqrt_denom = h_dot_in + eta * h_dot_out;

        return sqrt_base_color * (eta_factor * (1 - F) * D * G * eta * eta * fabs(h_dot_out * h_dot_in)) / 
               fmax(1e-6,fabs(dot(frame.n, dir_in)) * sqrt_denom * sqrt_denom);
    }
}

Real pdf_sample_bsdf_op::operator()(const DisneyGlass &bsdf) const {
    bool reflect = dot(vertex.geometric_normal, dir_in) *
                   dot(vertex.geometric_normal, dir_out) > 0;
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }

    Real eta = dot(vertex.geometric_normal, dir_in) > 0 ? bsdf.eta : 1 / bsdf.eta;
    assert(eta > 0);

    Vector3 half_vector;
    if (reflect) {
        half_vector = normalize(dir_in + dir_out);
    } else {
        half_vector = normalize(dir_in + dir_out * eta);
    }

    if (dot(half_vector, frame.n) < 0) {
        half_vector = -half_vector;
    }

    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    roughness = std::clamp(roughness, Real(0.01), Real(1));

    // Calculate PDF
    Real h_dot_in = dot(half_vector, dir_in);
    Real F = fresnel_dielectric(h_dot_in, eta);
    Vector3 hl= normalize(to_local(frame, half_vector));
    //Real D = GTR2(dot(frame.n, half_vector), roughness);
    //Real G_in = smith_masking_gtr2(to_local(frame, dir_in), roughness);
    Real D = GTR2aniso(hl, roughness, anisotropic);
    Real G_in = smith_masking_gtr2_aniso(to_local(frame, dir_in), roughness, anisotropic);

    if (reflect) {
        return (F * D * G_in) / fmax(1e-6, 4 * fabs(dot(frame.n, dir_in)));
    } else {
        Real h_dot_out = dot(half_vector, dir_out);
        Real sqrt_denom = h_dot_in + eta * h_dot_out;
        Real dh_dout = eta * eta * h_dot_out / fmax(1e-6, sqrt_denom * sqrt_denom*fabs(dot(frame.n, dir_in)));
        return (1 - F) * D * G_in * fabs(dh_dout * h_dot_in);
    }
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneyGlass &bsdf) const {
    Real eta = dot(vertex.geometric_normal, dir_in) > 0 ? bsdf.eta : 1 / bsdf.eta;
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }

    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    roughness = std::clamp(roughness, Real(0.01), Real(1));

    // Sample anisotropic visible normals
    Vector3 local_dir_in = to_local(frame, dir_in);
    //Vector3 local_micro_normal =sample_visible_normals(local_dir_in, roughness*roughness, rnd_param_uv);
    Vector3 local_micro_normal = sample_visible_normals_aniso(local_dir_in, roughness, anisotropic, rnd_param_uv);

    Vector3 half_vector = to_world(frame, local_micro_normal);
    if (dot(half_vector, frame.n) < 0) {
        half_vector = -half_vector;
    }

    Real h_dot_in = dot(half_vector, dir_in);
    Real F = fresnel_dielectric(h_dot_in, eta);

    if (rnd_param_w <= F) {
        // Reflection
        Vector3 reflected = normalize(-dir_in + 2 * dot(dir_in, half_vector) * half_vector);
        return BSDFSampleRecord{reflected, Real(0) /* eta */, roughness};
    } else {
        // Refraction
        Real h_dot_out_sq = 1 - (1 - h_dot_in * h_dot_in) / (eta * eta);
        if (h_dot_out_sq <= 0) {
            return {}; // Total internal reflection
        }
        if (h_dot_in < 0) {
            half_vector = -half_vector;
        }
        Real h_dot_out = sqrt(h_dot_out_sq);
        Vector3 refracted = -dir_in / eta + (fabs(h_dot_in) / eta - h_dot_out) * half_vector;
        return BSDFSampleRecord{refracted, eta, roughness};
    }
}

TextureSpectrum get_texture_op::operator()(const DisneyGlass &bsdf) const {
    return bsdf.base_color;
}
