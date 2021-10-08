from pathlib import Path
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os


if __name__ == "__main__":
    #mesh = o3dtut.get_bunny_mesh()
    print(":: Load point cloud")
    file_path = Path(__file__)
    folder = file_path.parent.parent
    folder = Path(os.path.join(folder, 'ply_files'))
    is_dir = folder.is_dir()

    #source_path = os.path.join(folder, '132716853518615948_only_face.ply')
    source_path = os.path.join(folder, 'only_face_doll6.ply')

    #target_path = os.path.join(folder, '132776633676957691_only_face_nir5.ply')
    pcd = o3d.io.read_point_cloud(source_path)
    #pcd = mesh.sample_points_poisson_disk(750)
    pcd.estimate_normals()
    #o3d.visualization.draw_geometries([pcd],point_show_normal=True)
    pcd.orient_normals_towards_camera_location(camera_location=np.array([0,0,0]))
    o3d.visualization.draw_geometries([pcd],point_show_normal=True)
    #exit(0)

    # # alpha shape method
    # alpha = 0.004
    # print(f"alpha={alpha:.3f}")
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    # mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    #
    # # ball pivoting method - requires normals
    # #pcd.estimate_normals()
    # pcd.orient_normals_consistent_tangent_plane(100)
    # radii = [0.005, 0.01, 0.02, 0.04]
    # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #     pcd, o3d.utility.DoubleVector(radii))
    # o3d.visualization.draw_geometries([rec_mesh])

    # Poisson surface reconstruction method - requires normals
    print('run Poisson surface reconstruction')

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9)
    print(mesh)
    o3d.visualization.draw_geometries([mesh])

    print('visualize densities')
    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    o3d.visualization.draw_geometries([density_mesh])

    print('remove low density vertices')
    vertices_to_remove = densities < np.quantile(densities, 0.08)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    print(mesh)
    mesh.transform(np.identity(4))
    o3d.visualization.draw_geometries([mesh])
    mesh_path = os.path.join(folder, 'only_face_doll6_mesh.obj')
    print(o3d.io.write_triangle_mesh(mesh_path, mesh))

    ##another ball pivoting method - requires normals
    # downpcd = pcd.voxel_down_sample(voxel_size=0.001)
    # distances = downpcd.compute_nearest_neighbor_distance()
    # avg_dist = np.mean(distances)
    # radius = 3 * avg_dist
    # bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(downpcd, o3d.utility.DoubleVector(
    #     [radius, radius * 2]))
    # bpa_mesh = bpa_mesh.filter_smooth_laplacian()
    # o3d.visualization.draw_geometries([bpa_mesh])
    # o3d.io.write_triangle_mesh(mesh_path, bpa_mesh)

