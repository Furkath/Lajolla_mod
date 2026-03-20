#!/usr/bin/env python3
"""Convert Mitsuba principled BSDF scenes to lajolla disney_bsdf format.

Usage: python convert_principled.py input.xml output.xml [--eta 1.45]
"""

import xml.etree.ElementTree as ET
import sys
import argparse
import copy
import re


def rgb_to_luminance(r, g, b):
    """Rec. 709 luminance."""
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def parse_rgb_string(value_str):
    """Parse '0.5 0.5 0.5' into (r, g, b)."""
    parts = value_str.strip().split()
    return tuple(float(x) for x in parts)


def convert_rgb_to_float(elem):
    """Convert an <rgb> element to a <float> element using luminance."""
    r, g, b = parse_rgb_string(elem.get("value"))
    lum = rgb_to_luminance(r, g, b)
    name = elem.get("name")
    elem.tag = "float"
    elem.set("value", f"{lum:.6f}")
    elem.set("name", name)
    # Remove any leftover attributes that don't belong on <float>
    return elem


def unwrap_twosided(bsdf_elem):
    """If bsdf is twosided, return the inner bsdf; otherwise return as-is."""
    if bsdf_elem.get("type") == "twosided":
        inner = bsdf_elem.find("bsdf")
        if inner is not None:
            # Carry over the id from the twosided wrapper
            outer_id = bsdf_elem.get("id")
            outer_name = bsdf_elem.get("name")
            if outer_id:
                inner.set("id", outer_id)
            if outer_name:
                inner.set("name", outer_name)
            return inner
    return bsdf_elem


def convert_bsdf(bsdf_elem, default_eta):
    """Convert a principled bsdf element to disneybsdf format."""
    bsdf_elem = unwrap_twosided(bsdf_elem)

    bsdf_type = bsdf_elem.get("type", "")
    if bsdf_type != "principled":
        return bsdf_elem

    # Rename type
    bsdf_elem.set("type", "disneybsdf")

    # Track which parameter names we've seen
    has_eta = False

    # Convert RGB tint parameters to scalar luminance
    rgb_to_scalar_params = {"spec_tint", "specular_tint", "specTint",
                            "sheen_tint", "sheenTint"}
    # Rename map
    rename_map = {
        "spec_trans": "specular_transmission",
        "specTrans": "specular_transmission",
        "spec_tint": "specular_tint",
        "specTint": "specular_tint",
    }

    for child in list(bsdf_elem):
        name = child.get("name", "")

        if name == "eta":
            has_eta = True

        # Convert RGB tints to scalar float via luminance
        if name in rgb_to_scalar_params and child.tag == "rgb":
            convert_rgb_to_float(child)

        # Rename parameters
        if name in rename_map:
            child.set("name", rename_map[name])

    # Add eta if not present
    if not has_eta:
        eta_elem = ET.SubElement(bsdf_elem, "float")
        eta_elem.set("name", "eta")
        eta_elem.set("value", f"{default_eta:.6f}")

    return bsdf_elem


def convert_scene(input_path, output_path, default_eta):
    # Preserve XML declaration and comments by reading raw
    tree = ET.parse(input_path)
    root = tree.getroot()

    # Process top-level bsdf elements
    bsdfs_to_replace = []
    for i, elem in enumerate(root):
        if elem.tag == "bsdf":
            bsdfs_to_replace.append((i, elem))

    for idx, bsdf_elem in bsdfs_to_replace:
        converted = convert_bsdf(bsdf_elem, default_eta)
        if converted is not bsdf_elem:
            root.remove(bsdf_elem)
            root.insert(idx, converted)

    # Also handle bsdf elements nested inside shapes
    for shape in root.iter("shape"):
        for bsdf in list(shape.findall("bsdf")):
            idx = list(shape).index(bsdf)
            converted = convert_bsdf(bsdf, default_eta)
            if converted is not bsdf:
                shape.remove(bsdf)
                shape.insert(idx, converted)

    # Write output
    ET.indent(tree, space="\t")
    tree.write(output_path, encoding="unicode", xml_declaration=False)

    # Prepend the xml content with scene version tag if needed
    print(f"Converted: {input_path} -> {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert Mitsuba principled BSDF to lajolla disneybsdf format")
    parser.add_argument("input", help="Input XML scene file")
    parser.add_argument("output", help="Output XML scene file")
    parser.add_argument("--eta", type=float, default=1.45,
                        help="Default IOR eta value (default: 1.45)")
    args = parser.parse_args()

    convert_scene(args.input, args.output, args.eta)


if __name__ == "__main__":
    main()
