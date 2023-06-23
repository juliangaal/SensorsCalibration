import open3d as o3d
pcd = o3d.io.read_point_cloud("0.pcd")
o3d.io.write_point_cloud("0.ply", pcd)
