"""
Blender script: Export selected mesh as PLY with per-loop (per-face-vertex) normals.
Vertices are split so each face corner gets its own vertex + normal.
This preserves sharp edges / custom normals that standard PLY export averages away.

Usage: Select the mesh object in Blender, then run this script.
       Change output_path below as needed.
"""

import bpy
import struct
import os

# ---- CONFIG ----
output_dir = bpy.path.abspath("//meshes/")  # relative to .blend file
# ----------------

def export_ply_perloop(obj, filepath):
    """Export mesh with per-loop normals (vertices split per face corner)."""
    import bmesh

    # Get evaluated (final) mesh with modifiers applied
    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()

    # Get world matrix for transforming positions and normals
    mat = obj.matrix_world
    mat_n = mat.inverted_safe().transposed().to_3x3()  # normal transform

    # Get UV layer
    uv_layer = mesh.uv_layers.active

    # Per-loop (corner) normals: Blender 4.x uses mesh.corner_normals
    corner_normals = mesh.corner_normals

    # Build vertex list: one vertex per loop (face corner)
    # Each vertex has: position, normal, uv
    verts = []
    faces = []
    vert_idx = 0

    for poly in mesh.polygons:
        face_indices = []
        for loop_idx in poly.loop_indices:
            loop = mesh.loops[loop_idx]
            # Position (world space)
            co = mat @ mesh.vertices[loop.vertex_index].co
            # Normal (world space, per-loop)
            no = mat_n @ corner_normals[loop_idx].vector
            no.normalize()
            # UV
            if uv_layer:
                uv = uv_layer.data[loop_idx].uv
            else:
                uv = (0.0, 0.0)

            # Blender: Z-up, Y-forward -> Target: Y-up, -Z-forward
            # (x, y, z) -> (x, z, -y)
            verts.append((co.x, co.z, -co.y, no.x, no.z, -no.y, uv[0], 1.0 - uv[1]))
            face_indices.append(vert_idx)
            vert_idx += 1

        # Triangulate: fan from first vertex
        for i in range(1, len(face_indices) - 1):
            faces.append((face_indices[0], face_indices[i], face_indices[i + 1]))

    # Write PLY (binary little-endian for speed)
    with open(filepath, 'wb') as f:
        # Header
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {len(verts)}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property float nx\n"
            "property float ny\n"
            "property float nz\n"
            "property float s\n"
            "property float t\n"
            f"element face {len(faces)}\n"
            "property list uchar int vertex_indices\n"
            "end_header\n"
        )
        f.write(header.encode('ascii'))

        # Vertex data
        for v in verts:
            f.write(struct.pack('<8f', *v))

        # Face data
        for face in faces:
            f.write(struct.pack('<B3i', 3, face[0], face[1], face[2]))

    eval_obj.to_mesh_clear()
    print(f"Exported {len(verts)} vertices, {len(faces)} faces to {filepath}")


# ---- MAIN ----
obj = bpy.context.active_object
if obj is None or obj.type != 'MESH':
    print("ERROR: Select a mesh object first!")
else:
    os.makedirs(output_dir, exist_ok=True)
    name = obj.name.replace(" ", "_")
    filepath = os.path.join(output_dir, f"{name}.ply")
    export_ply_perloop(obj, filepath)
    print(f"Done: {filepath}")
