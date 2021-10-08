# examples/Python/Advanced/colored_pointcloud_registration.py
import argparse
from pathlib import Path

import numpy as np
import copy
import open3d as o3d
import os


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='registration 2 ply files')
    parser.add_argument("--dir_path", required=True,
                        help="Path to ply files folder")

    args = parser.parse_args()

    folder = w_path = Path(args.dir_path)
    is_dir = folder.is_dir()

    source_path = os.path.join(folder, '132716853518615948_only_face.ply')
    target_path = os.path.join(folder, '132716853612617257_only_face.ply')
    print("1. Load two point clouds and show initial pose")
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)

    # draw initial alignment
    current_transformation = np.identity(4)
    draw_registration_result_original_color(source, target,
                                            current_transformation)

    # point to plane ICP
    current_transformation = np.identity(4)
    print("2. Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. Distance threshold 0.02.")
    source.estimate_normals()
    target.estimate_normals()
    result_icp = o3d.pipelines.registration.registration_icp(
        source, target, 0.02, current_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    print(result_icp)
    draw_registration_result_original_color(source, target,
                                            result_icp.transformation)

    # colored pointcloud registration
    # This is implementation of following paper
    # J. Park, Q.-Y. Zhou, V. Koltun,
    # Colored Point Cloud Registration Revisited, ICCV 2017
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = np.identity(4)
    print("3. Colored point cloud registration")
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        print("3-3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                    relative_rmse=1e-6,
                                                    max_iteration=iter))
        current_transformation = result_icp.transformation
        print(result_icp)
    #draw_registration_result_original_color(source, target,

# # examples/Python/Advanced/colored_pointcloud_registration.py
# import argparse
# from pathlib import Path

# import numpy as np
# import copy
# import open3d as o3d
# import os


# def draw_registration_result_original_color(source, target, transformation):
#     print(1)
#     source_temp = copy.deepcopy(source)
#     print(2)
#     source_temp.transform(transformation)
#     print(3)
#     o3d.visualization.draw_geometries([source_temp, target])
#     print(4)


# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description='registration 2 ply files')
#     # parser.add_argument("--dir_path", required=True,
#                         # help="Path to ply files folder")

#     # args = parser.parse_args()

#     # folder = w_path = Path(args.dir_path)
#     folder = Path("C:/Users/orend/OneDrive - Technion/Technion/Semester6/GIPProject/Hololens2_proj/ply_files")
#     is_dir = folder.is_dir()

#     source_path = os.path.join(folder, '132716853518615948_only_face.ply')
#     target_path = os.path.join(folder, '132716853612617257_only_face.ply')
#     print("1. Load two point clouds and show initial pose")
#     source = o3d.io.read_point_cloud(source_path)
#     target = o3d.io.read_point_cloud(target_path)

#     # draw initial alignment
#     current_transformation = np.identity(4)
#     draw_registration_result_original_color(source, target,
#                                             current_transformation)

#     # point to plane ICP
#     current_transformation = np.identity(4)
#     print("2. Point-to-plane ICP registration is applied on original point")
#     print("   clouds to refine the alignment. Distance threshold 0.02.")
#     source.estimate_normals()
#     target.estimate_normals()
#     result_icp = o3d.pipelines.registration.registration_icp(
#         source, target, 0.02, current_transformation,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane())
#     print(result_icp)
#     draw_registration_result_original_color(source, target,
#                                             result_icp.transformation)

#     # colored pointcloud registration
#     # This is implementation of following paper
#     # J. Park, Q.-Y. Zhou, V. Koltun,
#     # Colored Point Cloud Registration Revisited, ICCV 2017
#     voxel_radius = [0.04, 0.02, 0.01]
#     max_iter = [50, 30, 14]
#     current_transformation = np.identity(4)
#     print("3. Colored point cloud registration")
#     for scale in range(3):
#         iter = max_iter[scale]
#         radius = voxel_radius[scale]
#         print([iter, radius, scale])

#         print("3-1. Downsample with a voxel size %.2f" % radius)
#         source_down = source.voxel_down_sample(radius)
#         target_down = target.voxel_down_sample(radius)

#         print("3-2. Estimate normal.")
#         source_down.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
#         target_down.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

#         print("3-3. Applying colored point cloud registration")

#         result_icp = o3d.pipelines.registration.registration_colored_icp(
#             source_down, target_down, radius, current_transformation,
#             o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
#                                                     relative_rmse=1e-6,
#                                                     max_iteration=iter))
                                                    
#         current_transformation = result_icp.transformation
#         print(result_icp)
#     draw_registration_result_original_color(source, target,
# >>>>>>> Stashed changes:StreamRecorderConverter/registration_alg.py
#                                             result_icp.transformation)