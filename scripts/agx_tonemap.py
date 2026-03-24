"""
AgX tone mapping post-process for Lajolla renderer output.
Matches Blender's AgX Base Contrast (Look: None, Exposure: 0, Gamma: 1).

Usage:
    python agx_tonemap.py input.exr output.png
    python agx_tonemap.py input.exr output.png --exposure 1.5
    python agx_tonemap.py input.pfm output.png

Requires: pip install imageio[ffmpeg] imageio[pyav] numpy
  For EXR: pip install openexr (or imageio will use freeimage)
"""

import numpy as np
import sys
import argparse


def read_pfm(path):
    """Read a PFM file and return as float32 numpy array (H, W, 3)."""
    with open(path, 'rb') as f:
        header = f.readline().decode('ascii').strip()
        if header == 'PF':
            channels = 3
        elif header == 'Pf':
            channels = 1
        else:
            raise ValueError(f"Not a PFM file: {path}")
        dims = f.readline().decode('ascii').strip().split()
        width, height = int(dims[0]), int(dims[1])
        scale = float(f.readline().decode('ascii').strip())
        endian = '<' if scale < 0 else '>'
        data = np.frombuffer(f.read(), dtype=endian + 'f')
        if channels == 3:
            data = data.reshape((height, width, 3))
        else:
            data = data.reshape((height, width))
        # PFM stores bottom-to-top
        data = np.flipud(data).copy()
    return data


def read_exr(path):
    """Read an EXR file using OpenEXR directly."""
    try:
        import OpenEXR
        import Imath
        exr = OpenEXR.InputFile(path)
        header = exr.header()
        dw = header['dataWindow']
        width = dw.max.x - dw.min.x + 1
        height = dw.max.y - dw.min.y + 1
        pt = Imath.PixelType(Imath.PixelType.FLOAT)
        channels = header['channels']
        # Find RGB channel names (handle both 'R','G','B' and 'r','g','b')
        ch_names = list(channels.keys())
        r_name = next((c for c in ch_names if c.upper() == 'R'), ch_names[0])
        g_name = next((c for c in ch_names if c.upper() == 'G'), ch_names[min(1, len(ch_names)-1)])
        b_name = next((c for c in ch_names if c.upper() == 'B'), ch_names[min(2, len(ch_names)-1)])
        r_str = exr.channel(r_name, pt)
        g_str = exr.channel(g_name, pt)
        b_str = exr.channel(b_name, pt)
        r = np.frombuffer(r_str, dtype=np.float32).reshape((height, width))
        g = np.frombuffer(g_str, dtype=np.float32).reshape((height, width))
        b = np.frombuffer(b_str, dtype=np.float32).reshape((height, width))
        return np.stack([r, g, b], axis=-1)
    except ImportError:
        raise ImportError("OpenEXR not found. Install with: pip install openexr\n"
                          "Or render to PFM instead: ./lajolla.exe -o output.pfm scene.xml")


def read_image(path):
    """Read HDR image (EXR or PFM)."""
    if path.lower().endswith('.pfm'):
        return read_pfm(path)
    elif path.lower().endswith('.exr'):
        return read_exr(path)
    else:
        import imageio.v3 as iio
        return iio.imread(path).astype(np.float64)


def write_exr(path, img):
    """Write an EXR file using OpenEXR directly."""
    import OpenEXR
    import Imath
    img = img.astype(np.float32)
    h, w = img.shape[:2]
    header = OpenEXR.Header(w, h)
    header['channels'] = {
        'R': Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT)),
        'G': Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT)),
        'B': Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT)),
    }
    out = OpenEXR.OutputFile(path, header)
    out.writePixels({
        'R': img[:, :, 0].tobytes(),
        'G': img[:, :, 1].tobytes(),
        'B': img[:, :, 2].tobytes(),
    })
    out.close()


def write_hdr(path, img):
    """Write Radiance HDR (.hdr) file. No external dependencies."""
    img = np.clip(img, 0, None).astype(np.float32)
    h, w = img.shape[:2]
    with open(path, 'wb') as f:
        f.write(b'#?RADIANCE\n')
        f.write(b'FORMAT=32-bit_rle_rgbe\n\n')
        f.write(f'-Y {h} +X {w}\n'.encode())
        # Convert float RGB to RGBE encoding
        maxc = np.maximum(np.maximum(img[..., 0], img[..., 1]), img[..., 2])
        rgbe = np.zeros((h, w, 4), dtype=np.uint8)
        nonzero = maxc > 1e-38
        mantissa, exponent = np.frexp(maxc[nonzero])
        scale = mantissa * 256.0 / maxc[nonzero]
        rgbe[nonzero, 0] = np.clip(img[..., 0][nonzero] * scale, 0, 255).astype(np.uint8)
        rgbe[nonzero, 1] = np.clip(img[..., 1][nonzero] * scale, 0, 255).astype(np.uint8)
        rgbe[nonzero, 2] = np.clip(img[..., 2][nonzero] * scale, 0, 255).astype(np.uint8)
        rgbe[nonzero, 3] = (exponent + 128).astype(np.uint8)
        f.write(rgbe.tobytes())


def write_image(path, img):
    """Write image. PNG/JPG get 8-bit sRGB, EXR/PFM/HDR stay linear float."""
    if path.lower().endswith('.pfm'):
        h, w = img.shape[:2]
        with open(path, 'wb') as f:
            f.write(b'PF\n')
            f.write(f'{w} {h}\n'.encode())
            f.write(b'-1.0\n')
            np.flipud(img.astype(np.float32)).tofile(f)
    elif path.lower().endswith('.exr'):
        write_exr(path, img)
    elif path.lower().endswith('.hdr'):
        write_hdr(path, img)
    elif path.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
        import imageio.v3 as iio
        img8 = np.clip(img * 255, 0, 255).astype(np.uint8)
        iio.imwrite(path, img8)
    else:
        import imageio.v3 as iio
        iio.imwrite(path, img.astype(np.float32))


def agx_tonemap(img, exposure=0.0, apply_srgb=True):
    """
    Apply AgX tone mapping (Blender-style).
    Reference: three.js AgXToneMapping, verified against Blender's OCIO config.
    """
    img = img.astype(np.float64)

    # Apply exposure
    if exposure != 0.0:
        img *= 2.0 ** exposure

    r, g, b = img[..., 0], img[..., 1], img[..., 2]

    # Step 1: Linear sRGB -> Linear Rec.2020
    r2 = 0.6274039 * r + 0.3292830 * g + 0.0433131 * b
    g2 = 0.0690972 * r + 0.9195404 * g + 0.0113624 * b
    b2 = 0.0163914 * r + 0.0880133 * g + 0.8955953 * b

    # Step 2: AgX Inset Matrix (Rec.2020 -> AgX log input)
    ar = 0.856627153315983 * r2 + 0.0951212405381588 * g2 + 0.0482516061458583 * b2
    ag = 0.137318972929847 * r2 + 0.761241990602591  * g2 + 0.101439036467562  * b2
    ab = 0.11189821299995  * r2 + 0.0767994186031903 * g2 + 0.811302368396859  * b2

    # Step 3: Log2 encoding
    ar = np.maximum(ar, 1e-10)
    ag = np.maximum(ag, 1e-10)
    ab = np.maximum(ab, 1e-10)
    ar = np.log2(ar)
    ag = np.log2(ag)
    ab = np.log2(ab)

    # Step 4: Normalize to [0, 1] (16.5 stops range)
    AgxMinEv = -12.47393
    AgxMaxEv = 4.026069
    ar = np.clip((ar - AgxMinEv) / (AgxMaxEv - AgxMinEv), 0, 1)
    ag = np.clip((ag - AgxMinEv) / (AgxMaxEv - AgxMinEv), 0, 1)
    ab = np.clip((ab - AgxMinEv) / (AgxMaxEv - AgxMinEv), 0, 1)

    # Step 5: Sigmoid contrast approximation (6th order polynomial)
    def sigmoid(x):
        x2 = x * x
        x4 = x2 * x2
        return (15.5 * x4 * x2
                - 40.14 * x4 * x
                + 31.96 * x4
                - 6.868 * x2 * x
                + 0.4298 * x2
                + 0.1191 * x
                - 0.00232)

    ar = sigmoid(ar)
    ag = sigmoid(ag)
    ab = sigmoid(ab)

    # Step 6: AgX Outset Matrix
    or_ =  1.1271005818144368  * ar - 0.11060664309660323 * ag - 0.016493938717834573 * ab
    og  = -0.1413297634984383  * ar + 1.157823702216272   * ag - 0.016493938717834257 * ab
    ob  = -0.14132976349843826 * ar - 0.11060664309660294 * ag + 1.2519364065950405   * ab

    # Step 7: Linearize (pow 2.2)
    or_ = np.power(np.maximum(or_, 0), 2.2)
    og  = np.power(np.maximum(og,  0), 2.2)
    ob  = np.power(np.maximum(ob,  0), 2.2)

    # Step 8: Linear Rec.2020 -> Linear sRGB
    fr = 1.6605 * or_ - 0.5876 * og - 0.0728 * ob
    fg = -0.1246 * or_ + 1.1329 * og - 0.0083 * ob
    fb = -0.0182 * or_ - 0.1006 * og + 1.1187 * ob

    # Step 9: Linear sRGB -> sRGB gamma (only for display-referred formats like PNG)
    def linear_to_srgb(x):
        x = np.clip(x, 0, 1)
        return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1.0/2.4) - 0.055)

    if apply_srgb:
        result = np.stack([linear_to_srgb(fr), linear_to_srgb(fg), linear_to_srgb(fb)], axis=-1)
    else:
        result = np.stack([np.clip(fr, 0, 1), np.clip(fg, 0, 1), np.clip(fb, 0, 1)], axis=-1)
    return result


def main():
    parser = argparse.ArgumentParser(description='AgX tone mapping (Blender-style)')
    parser.add_argument('input', help='Input HDR image (EXR or PFM)')
    parser.add_argument('output', help='Output image (PNG, EXR, PFM)')
    parser.add_argument('--exposure', type=float, default=0.0, help='Exposure adjustment in stops (default: 0)')
    args = parser.parse_args()

    print(f"Reading {args.input}...")
    img = read_image(args.input)
    print(f"  Image size: {img.shape[1]}x{img.shape[0]}")

    # Apply sRGB gamma only for display formats (PNG/JPG), not for linear HDR (EXR/PFM)
    is_linear_output = args.output.lower().endswith(('.exr', '.pfm', '.hdr'))
    print(f"Applying AgX tone mapping (exposure={args.exposure}, sRGB={'no' if is_linear_output else 'yes'})...")
    result = agx_tonemap(img, exposure=args.exposure, apply_srgb=not is_linear_output)

    print(f"Writing {args.output}...")
    write_image(args.output, result)
    print("Done.")


if __name__ == '__main__':
    main()
