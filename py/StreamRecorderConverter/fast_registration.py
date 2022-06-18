# examples/Python/Advanced/global_registration.py
import shutil
from pathlib import Path
import open3d as o3d
import numpy as np
import copy
import os
import argparse
from StreamRecorderConverter.meshing_pcd import meshing_pcd
from datetime import datetime

file_path = Path(__file__)
folder = file_path.parent.parent
outputs_folder = Path(os.path.join(folder, 'outputs'))
ply_folder = Path(os.path.join(outputs_folder, 'ply_files'))
obj_folder = Path(os.path.join(outputs_folder, 'obj_files'))
txt_folder = Path(os.path.join(outputs_folder, 'txt_files'))


def draw_pcd(source, title='open3d'):
    source_temp = copy.deepcopy(source)
    source_temp.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([source_temp], window_name=title)
    # after_reg_path = os.path.join(saving_folder, 'after_reg_face_oren1.ply')
    # print(o3d.io.write_point_cloud(after_reg_path, source_temp))


def draw_registration_result(source, target, transformation=np.identity(4), title='open3d',
                             write_to_mesh=False):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([source_temp, target_temp, mesh_frame], window_name=title)
    if write_to_mesh:
        mesh_after_reg = meshing_pcd(source)
        mesh_after_reg.transform(transformation)
        after_reg_path = os.path.join(obj_folder, 'after_reg_mesh.obj')
        print(o3d.io.write_triangle_mesh(after_reg_path, mesh_after_reg))


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH (Fast Point Feature Histograms) feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


# sampling obj file to pcd
def sampling_obj_to_pcd(source_path):
    print('sampling obj to pcd:')
    source_mesh = o3d.io.read_triangle_mesh(source_path)
    mesh_pcd = source_mesh.sample_points_uniformly(number_of_points=3000)
    # mesh_pcd = source_mesh.sample_points_poisson_disk(number_of_points=3000, pcl=pcd)
    draw_pcd(mesh_pcd, title='source ply from mesh')
    return mesh_pcd


def prepare_dataset(voxel_size, source_name, target_name):
    print(":: Load two point clouds and disturb initial pose.")

    source_path = os.path.join(ply_folder, source_name)
    source = o3d.io.read_point_cloud(source_path)
    # From Open3D to numpy
    source_points_np = np.asarray(source.points)
    source_points_np[:, [2]] = source_points_np[:, [2]] * (-1)
    # From numpy to Open3D
    source.points = o3d.utility.Vector3dVector(source_points_np)
    #draw_pcd(source, title='original ply')

    ## sampling
    # source_mesh_path = os.path.join(obj_folder, 'source_name)
    # source = sampling_obj_to_pcd()

    target_path = os.path.join(ply_folder, target_name)
    target = o3d.io.read_point_cloud(target_path)
    # From Open3D to numpy
    target_points_np = np.asarray(target.points)
    target_points_np[:, [2]] = target_points_np[:, [2]] * (-1)
    # From numpy to Open3D
    target.points = o3d.utility.Vector3dVector(target_points_np)

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, False, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def refine_registration_point_to_plane(source, target, source_fpfh, target_fpfh, voxel_size,
                                       result_ransac_transformation):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    source.estimate_normals()
    target.estimate_normals()
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


def refine_registration_point_to_point(source, target, voxel_size
                                       , result_ransac_transformation):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='fast registration')
    parser.add_argument("--save_transformed_mesh", type=bool, required=False, default=True,
                        help="save transformed mesh?")
    parser.add_argument("--voxel_size", required=False, default=0.01, type=float,
                        help="voxel_size")

    args = parser.parse_args()

    save_transformed_mesh = args.save_transformed_mesh
    voxel_size = args.voxel_size

    obj_mesh = os.path.join(obj_folder, 'only_face_doll8_mesh.obj')
    obj_mesh_o3d = o3d.io.read_triangle_mesh(obj_mesh)
    o3d.visualization.draw_geometries([obj_mesh_o3d])

    #voxel_size = 0.05  # means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
        prepare_dataset(voxel_size, source_name='only_face_doll8.ply',
                        target_name='only_face_doll7.ply')
    draw_registration_result(source, target, np.identity(4),
                             title='Input')

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print("result_ransac:")
    print(result_ransac.transformation)
    # draw_registration_result(source_down, target_down,
    #                          result_ransac.transformation, title='Ransac Alg - Down Samples')

    draw_registration_result(source, target,
                             result_ransac.transformation,
                             title='Ransac Alg')
    print("Apply point-to-point ICP")
    result_point_to_point_icp = refine_registration_point_to_point(source, target,
                                                                   voxel_size, result_ransac.transformation)
    print(result_point_to_point_icp)
    print("result_point_to_point_icp Transformation is:")
    print(result_point_to_point_icp.transformation)
    print("")
    draw_registration_result(source, target, result_point_to_point_icp.transformation,
                             title='Point To Point ICP')

    print("\n\n")
    print("Apply point-to-plane ICP")
    result_icp_point_to_plane_icp = refine_registration_point_to_plane(source, target, source_fpfh, target_fpfh,
                                                                       voxel_size, result_ransac.transformation)
    print(result_icp_point_to_plane_icp)
    print("result_point_to_plane_icp Transformation is:")
    print(result_icp_point_to_plane_icp.transformation)
    print("")

    draw_registration_result(source, target, result_icp_point_to_plane_icp.transformation,
                             title='Point To Plane ICP', write_to_mesh=save_transformed_mesh)


# do registration from source to target, save the transformed mesh and return the path
def do_registration(source_path, target_path, source_mesh_path):
    source_name = os.path.basename(source_path)
    target_name = os.path.basename(target_path)
    source_mesh_name = os.path.basename(source_mesh_path)

    voxel_size = 0.01  # means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
        prepare_dataset(voxel_size, source_name=source_name, target_name=target_name)
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print("result_ransac:")
    print(result_ransac.transformation)

    # print("Apply point-to-point ICP")
    # result_point_to_point_icp = refine_registration_point_to_point(source, target,
    #                                                                voxel_size, result_ransac.transformation)

    print("\n\n")
    print("Apply point-to-plane ICP")
    result_icp_point_to_plane_icp = refine_registration_point_to_plane(source, target, source_fpfh, target_fpfh,
                                                                       voxel_size, result_ransac.transformation)
    print(result_icp_point_to_plane_icp)
    print("result_point_to_plane_icp Transformation is:")
    print(result_icp_point_to_plane_icp.transformation)
    print("")



    source_path_mesh = os.path.join(txt_folder, source_mesh_name)
    now = datetime.now()
    file_name = f'after_reg_mesh_{now.strftime("%d_%m_%Y__%H_%M_%S")}'
    source_path_mesh_after_reg = os.path.join(txt_folder, f'{file_name}_obj.obj')
    print(f"Reading {source_path_mesh}")
    source_mesh = o3d.io.read_triangle_mesh(source_path_mesh)
    source_temp_mesh = copy.deepcopy(source_mesh)
    source_temp_mesh.transform(result_icp_point_to_plane_icp.transformation)
    #draw_pcd(source_temp_mesh, title='mesh after reg')
    print('save transformed mesh status:')
    print(o3d.io.write_triangle_mesh(source_path_mesh_after_reg, source_temp_mesh))
    source_path_mesh_after_reg_to_send = os.path.join(txt_folder, f'{file_name}_txt.txt')
    shutil.copyfile(source_path_mesh_after_reg, source_path_mesh_after_reg_to_send)
    return source_path_mesh_after_reg_to_send, source_path_mesh_after_reg
