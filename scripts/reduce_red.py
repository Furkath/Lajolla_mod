#!/usr/bin/env python3
"""Reduce redness of an image while preserving overall luminance.

Usage: python reduce_red.py input.png [output.png] [--strength 0.5]

strength: 0.0 = no change, 1.0 = fully neutralize red shift
"""

import argparse
import numpy as np
from PIL import Image


def reduce_red(img_array, strength=0.5):
    """Shift hue away from red while preserving luminance.

    Works by computing per-pixel luminance, reducing the red channel,
    then rescaling all channels to restore the original luminance.
    """
    img = img_array.astype(np.float64) / 255.0

    # Rec. 709 luminance weights
    lum_weights = np.array([0.2126, 0.7152, 0.0722])
    original_lum = img @ lum_weights  # per-pixel luminance

    # Reduce red channel
    result = img.copy()
    result[:, :, 0] *= (1.0 - strength)

    # Compute new luminance
    new_lum = result @ lum_weights

    # Rescale to preserve original luminance
    # Avoid division by zero for black pixels
    scale = np.ones_like(new_lum)
    mask = new_lum > 1e-8
    scale[mask] = original_lum[mask] / new_lum[mask]

    result *= scale[:, :, np.newaxis]

    # Clamp to [0, 1]
    result = np.clip(result, 0, 1)

    return (result * 255).astype(np.uint8)


def main():
    parser = argparse.ArgumentParser(description="Reduce redness while preserving luminance")
    parser.add_argument("input", help="Input image path")
    parser.add_argument("output", nargs="?", default=None, help="Output image path (default: input_dered.png)")
    parser.add_argument("--strength", type=float, default=0.5,
                        help="How much to reduce red: 0.0=none, 1.0=max (default: 0.5)")
    args = parser.parse_args()

    output = args.output
    if output is None:
        stem, ext = args.input.rsplit(".", 1)
        output = f"{stem}_dered.{ext}"

    img = Image.open(args.input).convert("RGB")
    result = reduce_red(np.array(img), args.strength)
    Image.fromarray(result).save(output)
    print(f"Saved: {output} (strength={args.strength})")


if __name__ == "__main__":
    main()
