#!/usr/bin/env python3
"""Crop images to 2048x2048, centered horizontally, aligned to bottom.

Usage: python crop_center_bottom.py input_dir [output_dir] [--size 2048]
"""

import argparse
from pathlib import Path
from PIL import Image


def crop_center_bottom(img, size=2048):
    w, h = img.size

    # Horizontal: center
    left = (w - size) // 2
    # Vertical: bottom-aligned
    top = h - size

    # If image is smaller than target, pad with black
    if w < size or h < size:
        padded = Image.new(img.mode, (max(w, size), max(h, size)), 0)
        paste_x = (max(w, size) - w) // 2
        paste_y = max(h, size) - h
        padded.paste(img, (paste_x, paste_y))
        return crop_center_bottom(padded, size)

    return img.crop((left, top, left + size, top + size))


def main():
    parser = argparse.ArgumentParser(description="Crop images to square, centered horizontally, bottom-aligned")
    parser.add_argument("input_dir", help="Input directory with images")
    parser.add_argument("output_dir", nargs="?", default=None, help="Output directory (default: input_dir_cropped)")
    parser.add_argument("--size", type=int, default=2048, help="Target size (default: 2048)")
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir) if args.output_dir else input_dir.parent / (input_dir.name + "_cropped")
    output_dir.mkdir(parents=True, exist_ok=True)

    exts = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif", ".webp"}
    count = 0
    for f in sorted(input_dir.iterdir()):
        if f.suffix.lower() in exts:
            img = Image.open(f)
            cropped = crop_center_bottom(img, args.size)
            cropped.save(output_dir / f.name)
            count += 1
            print(f"  {f.name} ({img.size[0]}x{img.size[1]}) -> {args.size}x{args.size}")

    print(f"\nDone. {count} images saved to {output_dir}")


if __name__ == "__main__":
    main()
