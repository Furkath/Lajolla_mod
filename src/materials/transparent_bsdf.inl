/// Straight-through transparent BSDF (internal sub-lobe, not standalone).
/// True delta distribution, following Blender Cycles' approach:
///   eval = 0, pdf = 0 (delta has no continuous density)
///   sample returns -dir_in with is_delta = true
/// The path tracer detects is_delta and handles throughput (= 1) directly,
/// skipping MIS. Shadow rays use shadow_transmittance instead of occluded.

Spectrum eval_op::operator()(const TransparentBSDF &bsdf) const {
    // True delta: zero in the continuous sense.
    return make_zero_spectrum();
}

Real pdf_sample_bsdf_op::operator()(const TransparentBSDF &bsdf) const {
    // True delta: zero in the continuous sense.
    return Real(0);
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const TransparentBSDF &bsdf) const {
    // Pass straight through. eta=1 means no index of refraction change.
    return BSDFSampleRecord{-dir_in, Real(1) /* eta */, Real(0) /* roughness */, true /* is_delta */};
}

TextureSpectrum get_texture_op::operator()(const TransparentBSDF &bsdf) const {
    return make_constant_spectrum_texture(fromRGB(Vector3{1, 1, 1}));
}
