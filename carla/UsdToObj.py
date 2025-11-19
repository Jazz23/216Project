from pxr import Usd, UsdGeom, Gf
import os

INPUT_USD = "carla/building.usdc"
OUTPUT_DIR = "carla/exports"

stage = Usd.Stage.Open(INPUT_USD)
os.makedirs(OUTPUT_DIR, exist_ok=True)

def export_mesh_to_obj(mesh_prim, output_path):
    mesh = UsdGeom.Mesh(mesh_prim)
    
    # Get mesh attributes
    points = mesh.GetPointsAttr().Get()
    faceVertexCounts = mesh.GetFaceVertexCountsAttr().Get()
    faceVertexIndices = mesh.GetFaceVertexIndicesAttr().Get()

    if points is None or faceVertexCounts is None or faceVertexIndices is None:
        print(f"Skipping {mesh_prim.GetPath()} - missing geometry data")
        return

    # Compute and apply world transform to points
    xform_cache = UsdGeom.XformCache()
    transform_matrix = xform_cache.GetLocalToWorldTransform(mesh_prim)
    
    # Apply transform to each point
    transformed_points = []
    for p in points:
        vec4 = Gf.Vec4f(p[0], p[1], p[2], 1.0)  # Homogeneous coordinates
        transformed_vec4 = transform_matrix * vec4
        transformed_points.append(Gf.Vec3f(transformed_vec4[0], transformed_vec4[1], transformed_vec4[2]))

    with open(output_path, "w") as f:
        f.write(f"# Exported from USD prim {mesh_prim.GetPath()}\n")

        # Write vertices
        for p in transformed_points:
            f.write(f"v {p[0]} {p[1]} {p[2]}\n")

        # Write faces (OBJ uses 1-indexed)
        index_offset = 1
        idx = 0
        for count in faceVertexCounts:
            face = faceVertexIndices[idx: idx+count]
            face = [i+index_offset for i in face]  # OBJ indexing
            f.write("f " + " ".join(map(str, face)) + "\n")
            idx += count


# Iterate through all prims and export meshes
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        # Use the prim path to create a unique filename
        path_str = str(prim.GetPath()).replace('/', '_').replace('|', '_')
        out_path = os.path.join(OUTPUT_DIR, f"{path_str}.obj")
        print(f"Exporting: {prim.GetPath()} → {out_path}")
        export_mesh_to_obj(prim, out_path)

print("Done.")
