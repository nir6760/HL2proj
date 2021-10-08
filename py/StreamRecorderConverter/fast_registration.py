# examples/Python/Advanced/global_registration.py
from pathlib import Path
import open3d as o3d
import numpy as np
import copy
import os

file_path = Path(__file__)
containing_folder = file_path.parent.parent
containing_folder = Path(os.path.join(containing_folder,'ply_files'))

def draw_pcd(source, title='open3d'):

    source_temp = copy.deepcopy(source)
    source_temp.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([source_temp], window_name=title)
    #after_reg_path = os.path.join(saving_folder, 'after_reg_face_oren1.ply')
    #print(o3d.io.write_point_cloud(after_reg_path, source_temp))

def draw_registration_result(source, target, transformation=np.identity(4), title='open3d'):
    file_path = Path(__file__)
    saving_folder = file_path.parent.parent
    # folder = Path("C:/Users/orend/OneDrive - Technion/Technion/Semester6/GIPProject/Hololens2_proj/ply_files")
    saving_folder = Path(os.path.join(saving_folder, 'ply_files'))

    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name=title)
    after_reg_path = os.path.join(saving_folder, 'after_reg_face_nir5.ply')
    print(o3d.io.write_point_cloud(after_reg_path, source_temp))

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, source_name, target_name):
    print(":: Load two point clouds and disturb initial pose.")

    source_path = os.path.join(containing_folder, source_name)
    source = o3d.io.read_point_cloud(source_path)
    draw_pcd(source, title='original ply')

    source_mesh_path = os.path.join(containing_folder, 'only_face_doll6_mesh.obj')
    source_mesh = o3d.io.read_triangle_mesh(source_mesh_path)
    mesh_pcd = source_mesh.sample_points_uniformly(number_of_points=3000)
    #mesh_pcd = source_mesh.sample_points_poisson_disk(number_of_points=3000, pcl=pcd)
    draw_pcd(mesh_pcd, title='source ply from mesh')
    source = mesh_pcd

    target_path = os.path.join(containing_folder, target_name)
    target = o3d.io.read_point_cloud(target_path)
    #trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                         [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    #source.transform(trans_init)


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

def refine_registration_point_to_point(source, target, source_fpfh, target_fpfh, voxel_size
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
    voxel_size = 0.01  # means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
            prepare_dataset(voxel_size, source_name='only_face_doll6.ply',
                            target_name='only_face_doll5.ply')
    draw_registration_result(source, target, np.identity(4),
                             title='Input')
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print("result_ransac:")
    print(result_ransac.transformation)
    draw_registration_result(source_down, target_down,
                             result_ransac.transformation, title='Ransac Alg - Down Samples')

    draw_registration_result(source, target,
                             result_ransac.transformation,
                             title='Ransac Alg')
    print("Apply point-to-point ICP")
    result_point_to_point_icp = refine_registration_point_to_point(source, target, source_fpfh, target_fpfh,
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
                             title='Point To Plane ICP')

    source_path_mesh = os.path.join(containing_folder, 'mesh_face_nir5.obj')
    source_path_mesh_after_reg = os.path.join(containing_folder, 'after_reg_mesh_face_nir5.fbx')
    source_mesh = o3d.io.read_triangle_mesh(source_path_mesh)
    source_temp_mesh = copy.deepcopy(source_mesh)
    source_temp_mesh.transform(result_icp_point_to_plane_icp.transformation)
    draw_pcd(source_temp_mesh, title='mesh after reg')
    #o3d.io.write_triangle_mesh(source_path_mesh_after_reg, source_temp_mesh)

# do registration from source to target, save the transformed mesh and return the path
def do_registeration(source_path, target_path, source_mesh_path):
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
    # draw_registration_result(source_down, target_down,
    #                          result_ransac.transformation, title='Ransac Alg - Down Samples')

    # draw_registration_result(source, target,
    #                          result_ransac.transformation,
    #                          title='Ransac Alg')
    print("Apply point-to-point ICP")
    result_point_to_point_icp = refine_registration_point_to_point(source, target, source_fpfh, target_fpfh,
                                                                   voxel_size, result_ransac.transformation)
    print(result_point_to_point_icp)
    print("result_point_to_point_icp Transformation is:")
    print(result_point_to_point_icp.transformation)
    print("")
    # draw_registration_result(source, target, result_point_to_point_icp.transformation,
    #                          title='Point To Point ICP')

    print("\n\n")
    print("Apply point-to-plane ICP")
    result_icp_point_to_plane_icp = refine_registration_point_to_plane(source, target, source_fpfh, target_fpfh,
                                                                       voxel_size, result_ransac.transformation)
    print(result_icp_point_to_plane_icp)
    print("result_point_to_plane_icp Transformation is:")
    print(result_icp_point_to_plane_icp.transformation)
    print("")

    # draw_registration_result(source, target, result_icp_point_to_plane_icp.transformation,
    #                          title='Point To Plane ICP')

    source_path_mesh = os.path.join(containing_folder, source_mesh_name)
    source_path_mesh_after_reg = os.path.join(containing_folder, 'after_reg_mesh.obj')

    source_mesh = o3d.io.read_triangle_mesh(source_path_mesh)
    source_temp_mesh = copy.deepcopy(source_mesh)
    source_temp_mesh.transform(result_icp_point_to_plane_icp.transformation)
    #draw_pcd(source_temp_mesh, title='mesh after reg')
    print('save transformed mesh status:')
    print(o3d.io.write_triangle_mesh(source_path_mesh_after_reg, source_temp_mesh))
    p = Path(source_path_mesh_after_reg)
    p.rename(p.with_suffix('.txt'))
    return source_path_mesh_after_reg





