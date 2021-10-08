import open3d as o3d


mesh = o3d.io.read_triangle_mesh("C:/Users/orend/OneDrive - Technion/Technion/Semester6/GIPProject/Hololens2_proj/Technionface.obj")
pcd = mesh.PointCloud
pcd = mesh.sample_points_poisson_disk(30000)
o3d.visualization.draw_geometries([pcd])
o3d.io.write_point_cloud("C:/Users/orend/OneDrive - Technion/Technion/Semester6/GIPProject/Hololens2_proj/Technionface.pcd", pcd)
# source = o3d.io.read_point_cloud("C:/Users/orend/OneDrive - Technion/Technion/Semester6/GIPProject/Hololens2_proj/Technionface.obj")