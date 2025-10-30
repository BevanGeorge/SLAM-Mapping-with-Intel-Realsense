import open3d as o3d, numpy as np, sys, os

PLY_IN  = sys.argv[1] if len(sys.argv)>1 else "cloudlab710.ply"
OBJ_OUT = sys.argv[2] if len(sys.argv)>2 else "cloudlab710op.obj"

print("[1/6] Loading:", PLY_IN)
pcd = o3d.io.read_point_cloud(PLY_IN)
n = np.asarray(pcd.points).shape[0]
if n == 0:
    raise SystemExit("No points loaded. Check filename/path.")

print(f"[2/6] Downsample (0.03 m) from {n:,} pts ...")
pcd = pcd.voxel_down_sample(0.03)

print("[3/6] Estimate + orient normals ...")
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
pcd.orient_normals_consistent_tangent_plane(20)

print("[4/6] Poisson (depth=8) ...")
mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)

print("[5/6] Prune floaters + decimate ...")
dens = np.asarray(dens)
keep = dens > np.quantile(dens, 0.05)
idx = np.where(keep)[0]
mesh = mesh.select_by_index(idx)
mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=50000)
mesh.compute_vertex_normals()

print(f"[6/6] Write OBJ → {OBJ_OUT}")
ok = o3d.io.write_triangle_mesh(OBJ_OUT, mesh, write_triangle_uvs=False)
if not ok:
    raise SystemExit("Failed to write OBJ")
print("✅ Done.")
