"""
Blender script to export NPR material XML from visible objects.
Scans MMDShaderDev node groups for Base Tex, Toon Tex, and Sphere Tex
connections and generates <bsdf> XML entries.

Usage: Run in Blender's text editor or via scripting console.
"""

import bpy
import xml.etree.ElementTree as ET
from xml.dom import minidom
import os


def get_texture_filename(node_tree, input_name):
    """
    Given a node tree and an input socket name on the MMDShaderDev group node,
    trace back through the link to find the connected image texture node
    and return its image filename (basename only, prefixed with 'textures/').
    Returns None if no texture is connected.
    """
    # Find the MMDShaderDev group node
    group_node = None
    for node in node_tree.nodes:
        if node.type == 'GROUP' and node.node_tree and 'MMDShaderDev' in node.node_tree.name:
            group_node = node
            break

    if group_node is None:
        return None

    # Find the input socket by name
    input_socket = None
    for inp in group_node.inputs:
        if inp.name == input_name:
            input_socket = inp
            break

    if input_socket is None or not input_socket.links:
        return None

    # Trace back through the link
    linked_node = input_socket.links[0].from_node

    # The linked node might be another group node (e.g., "Mmd Base Tex")
    # We need to find the image texture inside it
    image = find_image_in_node(linked_node)

    if image is None:
        return None

    # Return just the filename prefixed with textures/
    return "textures/" + os.path.basename(image.filepath).replace("\\", "/")


def find_image_in_node(node):
    """
    Recursively find an image reference from a node.
    Handles direct Image Texture nodes and Group nodes.
    """
    # Direct image texture node
    if node.type == 'TEX_IMAGE' and node.image:
        return node.image

    # If it's a group node, search inside its node tree
    if node.type == 'GROUP' and node.node_tree:
        for inner_node in node.node_tree.nodes:
            if inner_node.type == 'TEX_IMAGE' and inner_node.image:
                return inner_node.image
            # Could recurse further, but MMD setups are typically shallow
            if inner_node.type == 'GROUP' and inner_node.node_tree:
                for deep_node in inner_node.node_tree.nodes:
                    if deep_node.type == 'TEX_IMAGE' and deep_node.image:
                        return deep_node.image

    return None


def sanitize_name(name):
    """Sanitize a material name for use as XML id/name attribute."""
    # Replace spaces and special chars with underscores
    return name.replace(" ", "_").replace(".", "_")


def build_bsdf_element(mat_name, diffuse_tex, toon_tex, sphere_tex):
    """Build a <bsdf> XML element for one material."""
    safe_name = "mat-" + sanitize_name(mat_name)

    bsdf = ET.Element("bsdf")
    bsdf.set("type", "npr")
    bsdf.set("id", safe_name)
    bsdf.set("name", safe_name)

    # diffuse_color texture
    if diffuse_tex:
        tex_diffuse = ET.SubElement(bsdf, "texture")
        tex_diffuse.set("type", "bitmap")
        tex_diffuse.set("name", "diffuse_color")
        s = ET.SubElement(tex_diffuse, "string")
        s.set("name", "filename")
        s.set("value", diffuse_tex)

    # toon_color texture
    if toon_tex:
        tex_toon = ET.SubElement(bsdf, "texture")
        tex_toon.set("type", "bitmap")
        tex_toon.set("name", "toon_color")
        s = ET.SubElement(tex_toon, "string")
        s.set("name", "filename")
        s.set("value", toon_tex)

    # sphere_color texture
    if sphere_tex:
        tex_sphere = ET.SubElement(bsdf, "texture")
        tex_sphere.set("type", "bitmap")
        tex_sphere.set("name", "sphere_color")
        s = ET.SubElement(tex_sphere, "string")
        s.set("name", "filename")
        s.set("value", sphere_tex)

    # Fixed parameters
    params = [
        ("float", "color_scale", "1"),
        ("rgb", "metallic_color", "0.00, 0.00, 0.00"),
        ("float", "Fac", "0.02"),
        ("float", "diffuse_roughness", "0.0"),
        ("float", "metallic_roughness", "1.0"),
    ]
    for tag, name, value in params:
        el = ET.SubElement(bsdf, tag)
        el.set("name", name)
        el.set("value", value)

    return bsdf


def prettify_xml(elem, indent="\t"):
    """Return a pretty-printed XML string with custom indentation."""
    rough = ET.tostring(elem, encoding="unicode")
    parsed = minidom.parseString(rough)
    pretty = parsed.toprettyxml(indent=indent)
    # Remove the XML declaration line
    lines = pretty.split("\n")
    return "\n".join(lines[1:])  # skip <?xml ...?>


def export_npr_xml(output_path=None):
    """
    Main export function. Iterates all visible objects, collects material info,
    and writes the XML file.
    """
    if output_path is None:
        output_path = bpy.path.abspath("//npr_materials.xml")

    # Collect materials from visible objects (avoid duplicates)
    processed_materials = set()
    bsdf_elements = []

    for obj in bpy.context.view_layer.objects:
        # Only process visible mesh objects
        if obj.type != 'MESH':
            continue
        if not obj.visible_get():
            continue

        if obj.data.materials is None:
            continue

        for mat in obj.data.materials:
            if mat is None:
                continue
            if mat.name in processed_materials:
                continue
            if not mat.use_nodes or mat.node_tree is None:
                continue

            processed_materials.add(mat.name)

            # Check if this material has an MMDShaderDev group node
            has_mmd = False
            for node in mat.node_tree.nodes:
                if node.type == 'GROUP' and node.node_tree and 'MMDShaderDev' in node.node_tree.name:
                    has_mmd = True
                    break

            if not has_mmd:
                continue

            # Get texture filenames
            diffuse_tex = get_texture_filename(mat.node_tree, "Base Tex")
            toon_tex = get_texture_filename(mat.node_tree, "Toon Tex")
            sphere_tex = get_texture_filename(mat.node_tree, "Sphere Tex")

            bsdf_el = build_bsdf_element(mat.name, diffuse_tex, toon_tex, sphere_tex)
            bsdf_elements.append(bsdf_el)

            print(f"  Material: {mat.name}")
            print(f"    diffuse: {diffuse_tex}")
            print(f"    toon:    {toon_tex}")
            print(f"    sphere:  {sphere_tex}")

    # Write XML
    with open(output_path, "w", encoding="utf-8") as f:
        for bsdf_el in bsdf_elements:
            f.write(prettify_xml(bsdf_el))
            f.write("\n")

    print(f"\nExported {len(bsdf_elements)} material(s) to: {output_path}")
    return output_path


# ---- Run ----
if __name__ == "__main__":
    # Change output path as needed
    output_file = bpy.path.abspath("//npr_materials.xml")
    export_npr_xml(output_file)
