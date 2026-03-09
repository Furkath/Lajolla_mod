/// Straight-through transparent BSDF (internal sub-lobe, not standalone).
/// Rays pass through unchanged. eval and pdf return matched values (1.0)
/// for the straight-through direction so throughput = eval/pdf = 1.0.
/// No geometric normal check — works from both sides of the surface.

Spectrum eval_op::operator()(const TransparentBSDF &bsdf) const {
    // Only non-zero for the exact straight-through direction.
    // When sampled, dir_out = -dir_in exactly, so dot = 1.0.
    // For NEE, dir_out is a light direction (almost never exactly -dir_in),
    // so this returns 0 — correct for a delta distribution.
    if (dot(dir_out, -dir_in) > Real(0.999)) {
        return make_const_spectrum(Real(1));
    }
    return make_zero_spectrum();
}

Real pdf_sample_bsdf_op::operator()(const TransparentBSDF &bsdf) const {
    if (dot(dir_out, -dir_in) > Real(0.999)) {
        return Real(1);
    }
    return Real(0);
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const TransparentBSDF &bsdf) const {
    // Pass straight through. eta=1 means no index of refraction change.
    // The path tracer's ray epsilon (get_intersection_epsilon) prevents
    // self-intersection with the same surface.
    return BSDFSampleRecord{-dir_in, Real(1) /* eta */, Real(0) /* roughness */};
}

TextureSpectrum get_texture_op::operator()(const TransparentBSDF &bsdf) const {
    return make_constant_spectrum_texture(fromRGB(Vector3{1, 1, 1}));
}
