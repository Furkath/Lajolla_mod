#include "../microfacet.h"

/// NPR BSDF: ((1-Fac)*diffuse + Fac*metal) * Fac2 + (1-Fac2) * transparent
/// Front face: full blending of opaque + transparent.
/// Back face: transparent only (opaque lobes are one-sided).

Spectrum eval_op::operator()(const NprBSDF &bsdf) const {
    Real fac2 = eval(bsdf.Fac2, vertex.uv, vertex.uv_screen_size, texture_pool);
    bool front_face = dot(vertex.geometric_normal, dir_in) > 0;

    Spectrum total = make_zero_spectrum();

    // Opaque lobes (front face only)
    if (front_face && fac2 > 0) {
        Real fac = eval(bsdf.Fac, vertex.uv, vertex.uv_screen_size, texture_pool);

        DisneyDiffuse diffuse_lobe = {bsdf.diffuse_color, bsdf.diffuse_roughness, bsdf.subsurface};
        DisneyTintMetal metal_lobe = {bsdf.metallic_color, bsdf.metallic_roughness, bsdf.anisotropic,
                                      bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta};

        Spectrum opaque = make_zero_spectrum();
        opaque += (1 - fac) * (*this)(diffuse_lobe);
        opaque += (*this)(metal_lobe);
        total += fac2 * opaque;
    }

    // Transparent lobe (both sides)
    if (fac2 < 1) {
        TransparentBSDF transparent_lobe = {};
        total += (1 - fac2) * (*this)(transparent_lobe);
    }

    return total;
}

Real pdf_sample_bsdf_op::operator()(const NprBSDF &bsdf) const {
    Real fac2 = eval(bsdf.Fac2, vertex.uv, vertex.uv_screen_size, texture_pool);
    bool front_face = dot(vertex.geometric_normal, dir_in) > 0;

    Real w_opaque = front_face ? fac2 : Real(0);
    Real w_transparent = Real(1) - fac2;
    Real w_sum = w_opaque + w_transparent;
    if (w_sum <= 0) return 0;

    Real pdf = 0;

    // Opaque lobes
    if (w_opaque > 0) {
        Real fac = eval(bsdf.Fac, vertex.uv, vertex.uv_screen_size, texture_pool);

        Real wo_diffuse = 1 - fac;
        Real wo_metal = 1;
        Real wo_sum = wo_diffuse + wo_metal;

        DisneyDiffuse diffuse_lobe = {bsdf.diffuse_color, bsdf.diffuse_roughness, bsdf.subsurface};
        DisneyTintMetal metal_lobe = {bsdf.metallic_color, bsdf.metallic_roughness, bsdf.anisotropic,
                                      bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta};

        Real opaque_pdf = 0;
        if (wo_sum > 0) {
            opaque_pdf += (wo_diffuse / wo_sum) * (*this)(diffuse_lobe);
            opaque_pdf += (wo_metal / wo_sum) * (*this)(metal_lobe);
        }
        pdf += (w_opaque / w_sum) * opaque_pdf;
    }

    // Transparent lobe
    if (w_transparent > 0) {
        TransparentBSDF transparent_lobe = {};
        pdf += (w_transparent / w_sum) * (*this)(transparent_lobe);
    }

    return fmax(Real(0), pdf);
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const NprBSDF &bsdf) const {
    Real fac2 = eval(bsdf.Fac2, vertex.uv, vertex.uv_screen_size, texture_pool);
    bool front_face = dot(vertex.geometric_normal, dir_in) > 0;

    Real w_opaque = front_face ? fac2 : Real(0);
    Real w_transparent = Real(1) - fac2;
    Real w_sum = w_opaque + w_transparent;
    if (w_sum <= 0) return {};

    Real p = rnd_param_w * w_sum;

    // Try opaque lobes first
    if (w_opaque > 0 && p < w_opaque) {
        Real fac = eval(bsdf.Fac, vertex.uv, vertex.uv_screen_size, texture_pool);

        Real wo_diffuse = 1 - fac;
        Real wo_metal = 1;
        Real wo_sum = wo_diffuse + wo_metal;
        if (wo_sum <= 0) return {};

        Real q = (p / w_opaque) * wo_sum;

        if (wo_diffuse > 0 && q < wo_diffuse) {
            DisneyDiffuse diffuse_lobe = {bsdf.diffuse_color, bsdf.diffuse_roughness, bsdf.subsurface};
            Real q_ = q / wo_diffuse;
            sample_bsdf_op sub_op = {dir_in, vertex, texture_pool, rnd_param_uv, q_, dir};
            return sub_op(diffuse_lobe);
        }
        q -= wo_diffuse;

        DisneyTintMetal metal_lobe = {bsdf.metallic_color, bsdf.metallic_roughness, bsdf.anisotropic,
                                      bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta};
        Real q_ = fmin(q / wo_metal, 1.0 - 1e-6);
        sample_bsdf_op sub_op = {dir_in, vertex, texture_pool, rnd_param_uv, q_, dir};
        return sub_op(metal_lobe);
    }

    // Transparent lobe
    TransparentBSDF transparent_lobe = {};
    sample_bsdf_op sub_op = {dir_in, vertex, texture_pool, rnd_param_uv, Real(0), dir};
    return sub_op(transparent_lobe);
}

TextureSpectrum get_texture_op::operator()(const NprBSDF &bsdf) const {
    return bsdf.diffuse_color;
}
