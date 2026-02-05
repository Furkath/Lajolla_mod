#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneyBSDF &bsdf) const {
    bool reflect = dot(vertex.geometric_normal, dir_in) *
                   dot(vertex.geometric_normal, dir_out) > 0;
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }
    // Homework 1: implement this!
    (void)reflect; // silence unuse warning, remove this when implementing hw
    if (dot(vertex.geometric_normal, dir_in) <= 0) {
        DisneyGlass glass_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
        Real weight = (1 - eval(bsdf.metallic, vertex.uv, vertex.uv_screen_size, texture_pool)) * eval(bsdf.specular_transmission, vertex.uv, vertex.uv_screen_size, texture_pool);
        return (*this)(glass_lobe) * weight; 
    }      

    Real metallic = eval(bsdf.metallic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real specTrans = eval(bsdf.specular_transmission, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real clearcoat = eval(bsdf.clearcoat, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real sheen_val = eval(bsdf.sheen, vertex.uv, vertex.uv_screen_size, texture_pool); 
        
    DisneyDiffuse diffuse_lobe = {bsdf.base_color, bsdf.roughness, bsdf.subsurface};
    DisneyTintMetal metal_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta};
    DisneyClearcoat clearcoat_lobe = {bsdf.clearcoat_gloss};
    DisneyGlass glass_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
    DisneySheen sheen_lobe = {bsdf.base_color, bsdf.sheen_tint};
        
    Spectrum total = make_zero_spectrum();
    total += (1 - specTrans) * (1 - metallic) * (*this)(diffuse_lobe);
    total += (1 - specTrans * (1 - metallic)) * (*this)(metal_lobe);
    total += (1 - metallic) * specTrans * (*this)(glass_lobe);
    total += 0.25 * clearcoat * (*this)(clearcoat_lobe);
    total += (1 - metallic) * sheen_val * (*this)(sheen_lobe);
    //total +=(1 - metallic) * (*this)(diffuse_lobe);
    //total +=metallic * (*this)(metal_lobe);
    
    return total;
    //return total/(2.0-specTrans-metallic+specTrans*metallic);
    //return make_zero_spectrum();
}

Real pdf_sample_bsdf_op::operator()(const DisneyBSDF &bsdf) const {
    bool reflect = dot(vertex.geometric_normal, dir_in) *
                   dot(vertex.geometric_normal, dir_out) > 0;
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }
    // Homework 1: implement this!
    (void)reflect; // silence unuse warning, remove this when implementing hw
    if (dot(vertex.geometric_normal, dir_in) <= 0) {
        DisneyGlass glass_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
        return (*this)(glass_lobe);
    }
   
    Real metallic = eval(bsdf.metallic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real specTrans = eval(bsdf.specular_transmission, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real clearcoat = eval(bsdf.clearcoat, vertex.uv, vertex.uv_screen_size, texture_pool);

    Real w_diffuse = (1 - metallic) * (1 - specTrans);
    Real w_metal = (1 - specTrans * (1 - metallic));
    Real w_glass = (1 - metallic) * specTrans;
    Real w_clearcoat = 0.25 * clearcoat;
    Real w_sum = w_diffuse + w_metal + w_glass + w_clearcoat;
    if (w_sum<=0) return 0;
    
    DisneyDiffuse diffuse_lobe = {bsdf.base_color, bsdf.roughness, bsdf.subsurface};
    DisneyTintMetal metal_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta};
    DisneyClearcoat clearcoat_lobe = {bsdf.clearcoat_gloss};
    DisneyGlass glass_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
    
    Real pdf = 0;

    //w_sum = w_diffuse + w_metal + w_glass;
    pdf += (w_diffuse/w_sum) * (*this)(diffuse_lobe);
    pdf += (w_metal/w_sum)  *  (*this)(metal_lobe);
    pdf += (w_glass/w_sum) * (*this)(glass_lobe);
    pdf += (w_clearcoat/w_sum) * (*this)(clearcoat_lobe);
    //pdf+=(1-metallic) * (*this)(diffuse_lobe);
    //pdf+=metallic * (*this)(metal_lobe); 
    
    //return pdf;
    return fmax(0.099, pdf);
    //return 0;
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneyBSDF &bsdf) const {
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }
    // Homework 1: implement this!
    if (dot(vertex.geometric_normal, dir_in) <= 0) {
        DisneyGlass glass_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
        return (*this)(glass_lobe);
    }  
  
    Real metallic = eval(bsdf.metallic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real specTrans = eval(bsdf.specular_transmission, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real clearcoat = eval(bsdf.clearcoat, vertex.uv, vertex.uv_screen_size, texture_pool);
     
    Real w_diffuse = (1 - metallic) * (1 - specTrans);
    Real w_metal = (1 - specTrans * (1 - metallic));
    Real w_glass = (1 - metallic) * specTrans;
    Real w_clearcoat = 0.25 * clearcoat;
    Real w_sum = w_diffuse + w_metal + w_glass + w_clearcoat;
    //Real w_sum = w_diffuse + w_metal + w_glass;
    //Real w_diffuse = (1 - metallic);
    //Real w_metal = metallic;
    //Real w_sum = 1.0;
    
    if (w_sum <=0 ) return {};

    Real p=rnd_param_w * w_sum;
 
    if (w_diffuse >0 && p < w_diffuse) {
        DisneyDiffuse diffuse_lobe = {bsdf.base_color, bsdf.roughness, bsdf.subsurface};
        Real p_ = p / w_diffuse;
        sample_bsdf_op sub_op = {dir_in, vertex, texture_pool, rnd_param_uv, p_, dir};   
        return sub_op(diffuse_lobe);
    }
    p -= w_diffuse; // Shift for next check

    if (w_metal>0 && p < w_metal) {
        DisneyTintMetal metal_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.specular, bsdf.specular_tint, bsdf.metallic, bsdf.eta};
        Real p_ = p / w_metal; 
        sample_bsdf_op sub_op = {dir_in, vertex, texture_pool, rnd_param_uv, p_, dir};
        return sub_op(metal_lobe);
    }
    p -= w_metal;   
   
    if (w_glass >0 && p < w_glass) {
        DisneyGlass glass_lobe = {bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
        Real p_ = p / w_glass;
        sample_bsdf_op sub_op = {dir_in, vertex, texture_pool, rnd_param_uv, p_, dir};
        return sub_op(glass_lobe);
    }
    p -= w_glass;
    // Clearcoat
    if (w_clearcoat>0){
    DisneyClearcoat clearcoat_lobe = {bsdf.clearcoat_gloss};
    Real p_ = fmin(p / w_clearcoat,1.0-1e-6); 
    sample_bsdf_op sub_op = {dir_in, vertex, texture_pool, rnd_param_uv, p_, dir};
    return sub_op(clearcoat_lobe);
    }
    return {};
}

TextureSpectrum get_texture_op::operator()(const DisneyBSDF &bsdf) const {
    return bsdf.base_color;
}
