#include "../microfacet.h"

/// NPR BSDF: ((1-Fac)*(diffuse*toon+sphere) + Fac*metal) * Fac2 + (1-Fac2) * transparent
/// toon_color and sphere_color are looked up with matcap UV (view-space normal) and added or multiplied with diffuse lobe.
/// Front face: full blending of opaque + transparent.
/// Back face: transparent only (opaque lobes are one-sided).

/// Compute matcap/toon UV from shading normal using the camera's view matrix.
/// Transforms normal to camera space, maps x,y from [-1,1] to [0,1].
/*inline Vector2 compute_toon_uv(const Vector3 &shading_normal, const Matrix4x4 &world_to_cam) {
    Vector3 cam_n = xform_vector(world_to_cam, shading_normal);
    Real u = std::clamp(cam_n.x * Real(0.5) + Real(0.5), Real(0), Real(1));
    Real v = std::clamp(-cam_n.y * Real(0.5) + Real(0.5), Real(0), Real(1));
    return Vector2{u, v};
}
*/
/// Old version: per-pixel view frame from ray direction (kept for potential future use).
/*
inline Vector2 compute_toon_uv_perpixel(const Vector3 &shading_normal, const Vector3 &dir_in) {
    Real len_sq = dot(dir_in, dir_in);
    if (len_sq < Real(1e-10)) {
        return Vector2{Real(0.5), Real(0.5)};
    }
    Vector3 view_z = dir_in / sqrt(len_sq);
    Vector3 world_up{0, 1, 0};
    if (fabs(dot(view_z, world_up)) > Real(0.999)) {
        world_up = Vector3{1, 0, 0};
    }
    Vector3 cross_val = cross(world_up, view_z);
    Real cross_len_sq = dot(cross_val, cross_val);
    if (cross_len_sq < Real(1e-10)) {
        return Vector2{Real(0.5), Real(0.5)};
    }
    Vector3 view_x = cross_val / sqrt(cross_len_sq);
    Vector3 view_y = cross(view_z, view_x);
    Real u = std::clamp(dot(shading_normal, view_x) * Real(0.5) + Real(0.5), Real(0), Real(1));
    Real v = std::clamp(dot(shading_normal, view_y) * Real(0.5) + Real(0.5), Real(0), Real(1));
    return Vector2{u, v};
}
*/

Spectrum eval_op::operator()(const NprBSDF &bsdf) const {
    Vector2 toon_uv = compute_toon_uv(vertex.shading_frame.n, bsdf.world_to_cam);
    // Toon/sphere textures use matcap UV (from camera-space normal), not surface UV.
    // Use footprint=0 (sharpest mip) since toon_uv varies smoothly and is unrelated
    // to surface UV screen derivatives.
    Real fac2 = eval(bsdf.Fac2, vertex.uv, vertex.uv_screen_size, texture_pool)
              * eval(bsdf.toon_alpha, toon_uv, vertex.uv_screen_size, texture_pool)
              * eval(bsdf.sphere_alpha, toon_uv, vertex.uv_screen_size, texture_pool);
    // Use face_front from true geometric normal (triangle winding),
    // matching Blender's "Backfacing" node behavior.
    bool front_face = vertex.face_front;
    //If backfacing, just see through (unless doublesided)
    if (!front_face && !bsdf.doublesided) fac2 = 0;

    Spectrum total = make_zero_spectrum();

    // Opaque lobes (front face only)
    if (fac2 > 0) {
        Real fac = eval(bsdf.Fac, vertex.uv, vertex.uv_screen_size, texture_pool);

        Spectrum toon = eval(bsdf.toon_color, toon_uv, vertex.uv_screen_size, texture_pool);
        Spectrum sphere = eval(bsdf.sphere_color, toon_uv, vertex.uv_screen_size, texture_pool);
        Spectrum toonshadow = eval(bsdf.toonshadow_color, toon_uv, vertex.uv_screen_size, texture_pool);
        Real shading_thresh = eval(bsdf.shading_threshold, vertex.uv, vertex.uv_screen_size, texture_pool);
        Spectrum spechighlight = eval(bsdf.specularhighlight_color, toon_uv, vertex.uv_screen_size, texture_pool);
        Real highlight_thresh = eval(bsdf.highlight_threshold, vertex.uv, vertex.uv_screen_size, texture_pool);
        Spectrum rimlight = eval(bsdf.rimlight_color, toon_uv, vertex.uv_screen_size, texture_pool);
        Real rimlight_thresh = eval(bsdf.rimlight_threshold, vertex.uv, vertex.uv_screen_size, texture_pool);

        NprDiffuse diffuse_lobe = {bsdf.diffuse_color, bsdf.diffuse_roughness, bsdf.subsurface,
                                   toon, sphere, toonshadow, shading_thresh, spechighlight, highlight_thresh, 
                                   rimlight, rimlight_thresh, bsdf.virtual_light, bsdf.doublesided};
        NprTintMetal metal_lobe = {bsdf.metallic_color, bsdf.metallic_roughness, bsdf.anisotropic,
                                      bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta,
                                      bsdf.doublesided};

        Spectrum opaque = make_zero_spectrum();
        opaque += (1 - fac) * bsdf.color_scale * (*this)(diffuse_lobe);
        opaque += fac * (*this)(metal_lobe);
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
    Vector2 toon_uv = compute_toon_uv(vertex.shading_frame.n, bsdf.world_to_cam);
    Real fac2 = eval(bsdf.Fac2, vertex.uv, vertex.uv_screen_size, texture_pool)
              * eval(bsdf.toon_alpha, toon_uv, vertex.uv_screen_size, texture_pool)
              * eval(bsdf.sphere_alpha, toon_uv, vertex.uv_screen_size, texture_pool);
    // Use face_front from true geometric normal (triangle winding),
    // matching Blender's "Backfacing" node behavior.
    bool front_face = vertex.face_front;
    //If backfacing, just see through (unless doublesided)
    if (!front_face && !bsdf.doublesided) fac2 = 0;

    Real w_opaque = fac2;
    Real w_transparent = Real(1) - fac2;
    Real w_sum = w_opaque + w_transparent;
    if (w_sum <= 0) return 0;

    Real pdf = 0;

    // Opaque lobes
    if (w_opaque > 0) {
        Real fac = eval(bsdf.Fac, vertex.uv, vertex.uv_screen_size, texture_pool);

        Real wo_diffuse = 1 - fac;
        Real wo_metal = fac;
        Real wo_sum = wo_diffuse + wo_metal;

        NprDiffuse diffuse_lobe = {bsdf.diffuse_color, bsdf.diffuse_roughness, bsdf.subsurface,
                                   make_zero_spectrum(), make_zero_spectrum(), make_zero_spectrum(), 
                                   Real(-1.0), make_zero_spectrum(), Real(1.0), 
                                   make_zero_spectrum(), Real(1.0), bsdf.virtual_light, bsdf.doublesided};
        NprTintMetal metal_lobe = {bsdf.metallic_color, bsdf.metallic_roughness, bsdf.anisotropic,
                                      bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta,
                                      bsdf.doublesided};

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

    return fmax(Real(0.001), pdf);
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const NprBSDF &bsdf) const {
    Vector2 toon_uv = compute_toon_uv(vertex.shading_frame.n, bsdf.world_to_cam);
    Real fac2 = eval(bsdf.Fac2, vertex.uv, vertex.uv_screen_size, texture_pool)
              * eval(bsdf.toon_alpha, toon_uv, vertex.uv_screen_size, texture_pool)
              * eval(bsdf.sphere_alpha, toon_uv, vertex.uv_screen_size, texture_pool);
    // Use face_front from true geometric normal (triangle winding),
    // matching Blender's "Backfacing" node behavior.
    bool front_face = vertex.face_front;
    //If backfacing, just see through (unless doublesided)
    if (!front_face && !bsdf.doublesided) fac2 = 0;

    Real w_opaque = fac2;
    Real w_transparent = Real(1) - fac2;
    Real w_sum = w_opaque + w_transparent;
    if (w_sum <= 0) return {};

    Real p = rnd_param_w * w_sum;

    // Try opaque lobes first
    if (w_opaque > 0 && p < w_opaque) {
        Real fac = eval(bsdf.Fac, vertex.uv, vertex.uv_screen_size, texture_pool);

        Real wo_diffuse = 1 - fac;
        Real wo_metal = fac;
        Real wo_sum = wo_diffuse + wo_metal;
        if (wo_sum <= 0) return {};

        Real q = (p / w_opaque) * wo_sum;

        if (wo_diffuse > 0 && q < wo_diffuse) {
            NprDiffuse diffuse_lobe = {bsdf.diffuse_color, bsdf.diffuse_roughness, bsdf.subsurface,
                                       make_zero_spectrum(), make_zero_spectrum(), make_zero_spectrum(), 
                                       Real(-1.0), make_zero_spectrum(), Real(1.0), 
                                       make_zero_spectrum(), Real(1.0), bsdf.virtual_light, bsdf.doublesided};
            Real q_ = q / wo_diffuse;
            sample_bsdf_op sub_op = {dir_in, vertex, texture_pool, rnd_param_uv, q_, dir};
            return sub_op(diffuse_lobe);
        }
        q -= wo_diffuse;

        NprTintMetal metal_lobe = {bsdf.metallic_color, bsdf.metallic_roughness, bsdf.anisotropic,
                                      bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta,
                                      bsdf.doublesided};
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
