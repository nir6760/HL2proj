import multiprocessing
import argparse
from pathlib import Path

import numpy as np
import cv2
import open3d as o3d
import os
import shutil
import copy
from datetime import datetime

from StreamRecorderConverter.project_hand_eye_to_pv import load_pv_data, match_timestamp
from StreamRecorderConverter.utils import extract_tar_file, load_lut, DEPTH_SCALING_FACTOR, project_on_depth
from StreamRecorderConverter.save_pclouds import save_output_txt_files

sensor_name = "Depth AHaT"


def shift_img(img, folder):
    height, width = img.shape
    # img2 = np.zeros((height, width), np.uint16)
    # img3 = np.zeros((height, width), np.uint16)
    img2 = img.byteswap()
    # for h in range(height):
    #     for w in range(width):
    #         img2[h, w] = ((img[h, w] & 0b1111111100000000) >> 8) + ((img[h, w] & 0b0000000011111111) << 8)
    #         if img[h, w] != 0:
    #             img3[h, w] =img[h, w] + 2500
    cv2.imwrite(os.path.join(folder, sensor_name, 'shifted_img.pgm'), img2)
    # cv2.imwrite(os.path.join(folder, 'Depth Long Throw', 'plus_1500.pgm'), img3)
    pass


# helper for delete too far points
def img_show(center_point, radius):
    # Read point cloud from PLY
    pcd1 = o3d.io.read_point_cloud("1.ply")
    points = np.asarray(pcd1.points)

    # Sphere center and radius
    # center = np.array([1.586, -8.436, -0.242])

    # Calculate distances to center, set new points
    distances = np.linalg.norm(points - center_point, axis=1)
    pcd1.points = o3d.utility.Vector3dVector(points[distances <= radius])

    # Write point cloud out
    o3d.io.write_point_cloud("out.ply", pcd1)


# delete only txt files from dir content
def delete_txt_from_dir_content(path):
    for root, dirs, files in os.walk(path):
        for f in files:
            if f.endswith('txt'):
                os.unlink(os.path.join(root, f))


# delete dir content
def delete_dir_content(path):
    for root, dirs, files in os.walk(path):
        for f in files:
            os.unlink(os.path.join(root, f))
        for d in dirs:
            shutil.rmtree(os.path.join(root, d))


# write txt telemetry for pv
def write_pv_txt(folder, video_reciver_header):
    # datetime object containing current date and time
    # now = datetime.timestamp()
    file_txt_name = '_pv.txt'
    txt_path = os.path.join(folder, file_txt_name)
    with open(txt_path, 'w') as f:
        f.write(str(video_reciver_header.principalPoint_x)
                + ', ' +
                str(video_reciver_header.principalPoint_y)
                + ', ' +
                str(video_reciver_header.ImageWidth)
                + ', ' +
                str(video_reciver_header.ImageHeight) + '\n')

        f.write(str(video_reciver_header.Timestamp) + ', ' +
                str(video_reciver_header.fx) + ', ' +
                str(video_reciver_header.fy) + ', ' +

                str(video_reciver_header.PVtoWorldtransformM11) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM21) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM31) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM41) + ', ' +

                str(video_reciver_header.PVtoWorldtransformM12) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM22) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM32) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM42) + ', ' +

                str(video_reciver_header.PVtoWorldtransformM13) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM23) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM33) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM43) + ', ' +

                str(video_reciver_header.PVtoWorldtransformM14) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM24) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM34) + ', ' +
                str(video_reciver_header.PVtoWorldtransformM44))


# write txt telemetry for depth
def write_depth_txt(folder, ahat_receiver_header):
    rig2world_name = r'{}_rig2world.txt'.format(sensor_name)
    txt_path = os.path.join(folder, rig2world_name)
    with open(txt_path, 'w') as f:
        f.write(str(ahat_receiver_header.Timestamp) + ', ' +

                str(ahat_receiver_header.rig2worldTransformM11) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM21) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM31) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM41) + ', ' +

                str(ahat_receiver_header.rig2worldTransformM12) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM22) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM32) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM42) + ', ' +

                str(ahat_receiver_header.rig2worldTransformM13) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM23) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM33) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM43) + ', ' +

                str(ahat_receiver_header.rig2worldTransformM14) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM24) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM34) + ', ' +
                str(ahat_receiver_header.rig2worldTransformM44))


# save data in correct folder before registration
def save_data_for_registration(video_receiver_header, ahat_receiver_header,
                               video_receiver_frame, ahat_receiver_frame,
                               folder):
    write_pv_txt(folder, video_receiver_header)
    if ahat_receiver_frame is not None and ahat_receiver_frame is not None:
        write_depth_txt(folder, ahat_receiver_header)
        save_imgs_for_registration(str(video_receiver_header.Timestamp), str(ahat_receiver_header.Timestamp),
                                   video_receiver_frame, ahat_receiver_frame, folder)


# imwrite rgb and depth frames
def save_imgs_for_registration(timestamp_pv, timestamp_depth
                               , video_receiver_frame, ahat_receiver_frame, folder):
    # pv
    # delete content PV folder
    pv_img_dir_path = os.path.join(folder, 'PV')
    delete_dir_content(pv_img_dir_path)
    pv_img_path = os.path.join(pv_img_dir_path, timestamp_pv + '.png')
    cv2.imwrite(pv_img_path, video_receiver_frame)
    # depth
    # delete content Depth Long Throw
    depth_img_dir_path = os.path.join(folder, 'Depth AHaT')
    delete_dir_content(depth_img_dir_path)
    depth_img_path = os.path.join(depth_img_dir_path, timestamp_depth + '.pgm')
    # np swap bytes
    ahat_receiver_frame_swaped = ahat_receiver_frame.byteswap()
    cv2.imwrite(depth_img_path, ahat_receiver_frame_swaped)


# helper for showing image
def img_show(img):
    # Window name in which image is displayed
    window_name = 'image'
    # Using cv2.imshow() method
    # Displaying the image
    cv2.imshow(window_name, img)

    # waits for user to press any key
    # (this is necessary to avoid Python kernel form crashing)
    cv2.waitKey(0)

    # closing all open windows
    cv2.destroyAllWindows()


# helper for detect face and mask, return pixel of the center of the face
def detect_and_mask(img):
    plus = 0
    minus = 0
    # img_show(img)  # 1/3
    # Load the cascade
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    # Detect faces
    faces = face_cascade.detectMultiScale(img, 1.1, 4)
    face_detected = False
    # Draw rectangle around the faces
    x = y = w = h = 0
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x + plus, y + plus), (x + w - minus, y + h - minus), (255, 0, 0), 2)
        face_detected = True
        # top left and bottum right
    # Using cv2.imshow() method
    # Displaying the image
    # img_show(img)  # 2/3
    # mask
    mask = None
    if face_detected:
        mask = np.zeros(img.shape, np.uint8)
        mask[y:y + h, x:x + w] = img[y:y + h, x:x + w]
    # img_show(mask)  # 3/3
    return mask, int((x + w) / 2), int((y + h) / 2), face_detected


def find_index_center_face(xy, x_center_face, y_center_face):
    # maybe the x and the y is supposed to be oposite
    result = np.where((xy[:, 1] == x_center_face) & (xy[:, 0] == y_center_face))
    if result[0].size == 1:
        return result[0][0]  # this is index of closed enough point to the center of the face
    # closest = np.argmin(xy, key=lambda v: (v[0] - x_center_face) ** 2 + (v[1] - y_center_face) ** 2)
    xy_ax0 = xy[:, 0] - y_center_face
    xy_ax0 = xy_ax0.reshape((len(xy), 1))
    xy_ax1 = xy[:, 1] - x_center_face
    xy_ax1 = xy_ax1.reshape((len(xy), 1))
    xy_after = np.hstack((xy_ax0, xy_ax1))
    val = np.argmin(xy_after)
    # closest = np.where((xy_after[:, 0] == val[0]) & (xy_after[:, 1] == val[1]))
    # return closest[0][0]
    return val
    # if result[0].size == 0:
    #     for i_x in range(-10, 10, 1):
    #         x_center_face += i_x
    #         for i_y in range(-10, 10, 1):
    #             y_center_face += i_y
    #             result = np.where((xy[:, 1] == x_center_face) & (xy[:, 0] == y_center_face))
    #             if result[0].size == 1:
    #                 return result[0][0]  # this is index of closed enough point to the center of the face
    #         y_center_face = first_y
    # raise ValueError('didnt find pixel closed enough to the center of the image')


def project_on_pv_nir(points, pv_img, pv2world_transform, focal_length, principal_point,
                      x_center_face, y_center_face):
    height, width, _ = pv_img.shape

    homog_points = np.hstack((points, np.ones(len(points)).reshape((-1, 1))))
    world2pv_transform = np.linalg.inv(pv2world_transform)
    points_pv = (world2pv_transform @ homog_points.T).T[:, :3]

    intrinsic_matrix = np.array([[focal_length[0], 0, principal_point[0]], [
        0, focal_length[1], principal_point[1]], [0, 0, 1]])
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    # xy : this is a dictionary,  point_cloud_id - [x,y] pixel on pv
    xy, _ = cv2.projectPoints(points_pv, rvec, tvec, intrinsic_matrix, None)
    xy = np.squeeze(xy)
    xy[:, 0] = width - xy[:, 0]
    xy = np.around(xy).astype(int)

    rgb = np.zeros_like(points)
    width_check = np.logical_and(0 <= xy[:, 0], xy[:, 0] < width)
    height_check = np.logical_and(0 <= xy[:, 1], xy[:, 1] < height)
    valid_ids = np.where(np.logical_and(width_check, height_check))[0]

    z = points_pv[valid_ids, 2]
    # xy now is only the points which can be at the rgb photo
    xy = xy[valid_ids, :]
    index_center_face = find_index_center_face(xy, x_center_face, y_center_face)
    xyz_center_face = points[index_center_face]
    depth_image = np.zeros((height, width))
    for i, p in enumerate(xy):
        depth_image[p[1], p[0]] = z[i]

    colors = pv_img[xy[:, 1], xy[:, 0], :]
    # colors- this is a dictionary,  point_cloud_id - [r,g,b] color of point on pv
    rgb[valid_ids, :] = colors[:, ::-1] / 255.

    return rgb, depth_image, xyz_center_face


def save_single_pcloud(shared_dict,
                       path,
                       folder,
                       save_in_cam_space,
                       lut,
                       has_pv,
                       focal_lengths,
                       principal_point,
                       rig2world_transforms,
                       rig2cam,
                       pv_timestamps,
                       pv2world_transforms,
                       discard_no_rgb,
                       clamp_min,
                       clamp_max,
                       depth_path_suffix,
                       disable_project_pinhole,
                       objects_folder,
                       radius_cut=0.5
                       ):
    xyz_center_face = None
    suffix = '_cam' if save_in_cam_space else ''
    output_path = str(path)[:-4] + f'{suffix}.ply'

    #    if Path(output_path).exists():
    #        print(output_path + ' is already exists, skip generating this pclouds')
    #        return

    print(".", end="", flush=True)

    # extract the timestamp for this frame
    timestamp = extract_timestamp(path.name.replace(depth_path_suffix, ''))
    # load depth img
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    # shift_img(img, folder)
    # save_nparr_in_txtfile(img, folder, 'depth_img')
    height, width = img.shape
    assert len(lut) == width * height

    # Clamp values if requested
    if clamp_min > 0 and clamp_max > 0:
        # Convert crop values to mm
        clamp_min = clamp_min * 1000.
        clamp_max = clamp_max * 1000.
        # Clamp depth values
        img[img < clamp_min] = 0
        img[img > clamp_max] = 0

    # img_show(img)

    # Get xyz points in camera space
    points = get_points_in_cam_space(img, lut)

    if save_in_cam_space:
        save_ply_in_file(output_path, points, rgb=None)
        # print('Saved %s' % output_path)
    else:
        if rig2world_transforms and (timestamp in rig2world_transforms):
            # if we have the transform from rig to world for this frame,
            # then put the point clouds in world space
            rig2world = rig2world_transforms[timestamp]
            # print('Transform found for timestamp %s' % timestamp)
            xyz, cam2world_transform = cam2world(points, rig2cam, rig2world)

            rgb = None
            if has_pv:
                # if we have pv, get vertex colors
                # get the pv frame which is closest in time
                target_id = match_timestamp(timestamp, pv_timestamps)
                pv_ts = pv_timestamps[target_id]
                rgb_path = str(folder / 'PV' / f'{pv_ts}.png')
                assert Path(rgb_path).exists()
                pv_img = cv2.imread(rgb_path)

                pv_img_after, x_center_face, y_center_face, face_detected = detect_and_mask(pv_img)
                if face_detected:
                    print("Detected a face")
                    # Project from depth to pv going via world space
                    # index_center_face is the index of point cloud id at the pixel of the center of the face in color image
                    rgb, depth, xyz_center_face = project_on_pv_nir(
                        xyz, pv_img_after, pv2world_transforms[target_id],
                        focal_lengths[target_id], principal_point, x_center_face, y_center_face)

                    # Project depth on virtual pinhole camera and save corresponding
                    # rgb image inside <workspace>/pinhole_projection folder
                    if not disable_project_pinhole:
                        pinhole_folder = os.path.join(folder, 'pinhole_projection')
                        # Create virtual pinhole camera
                        scale = 1
                        width = 320 * scale
                        height = 288 * scale
                        focal_length = 200 * scale
                        intrinsic_matrix = np.array([[focal_length, 0, width / 2.],
                                                     [0, focal_length, height / 2.],
                                                     [0, 0, 1.]])
                        rgb_proj, depth = project_on_depth(
                            points, rgb, intrinsic_matrix, width, height)

                        # # Save depth image
                        depth_proj_name = 'depth' + '\\' + f'{pv_ts}.png'
                        depth_proj_path = str(depth_proj_name)[:-4] + f'{suffix}_proj.png'
                        pinhole_folder = Path(pinhole_folder)
                        depth_proj_path = str(pinhole_folder / depth_proj_path)
                        depth = (depth * DEPTH_SCALING_FACTOR).astype(np.uint16)
                        delete_dir_content(str(pinhole_folder / 'depth'))
                        cv2.imwrite(depth_proj_path, (depth).astype(np.uint16))
                        #
                        # # Save rgb image
                        rgb_proj_name = 'rgb' + '\\' + f'{pv_ts}.png'
                        rgb_proj_path = str(rgb_proj_name)[:-4] + f'{suffix}_proj.png'
                        rgb_proj_path = str(pinhole_folder / rgb_proj_path)
                        delete_dir_content(str(pinhole_folder / 'rgb'))
                        cv2.imwrite(rgb_proj_path, rgb_proj)

                        # Save virtual pinhole information inside calibration.txt
                        intrinsic_path = Path(str(pinhole_folder / 'calibration.txt'))
                        intrinsic_list = [intrinsic_matrix[0, 0], intrinsic_matrix[1, 1],
                                          intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]]
                        with open(str(intrinsic_path), "w") as p:
                            p.write(f"{intrinsic_list[0]} \
                                    {intrinsic_list[1]} \
                                    {intrinsic_list[2]} \
                                    {intrinsic_list[3]} \n")

                        # Create rgb and depth paths
                        rgb_parts = Path(rgb_proj_path).parts[2:]
                        rgb_tmp = Path(rgb_parts[-2]) / Path(rgb_parts[-1])
                        depth_parts = Path(depth_proj_path).parts[2:]
                        depth_tmp = Path(depth_parts[-2]) / Path(depth_parts[-1])

                        # Compute camera center
                        camera_center = cam2world_transform @ np.array([0, 0, 0, 1])

                        # Save depth, rgb, camera center, extrinsics inside shared dictionary
                        shared_dict[path.stem] = [depth_tmp, rgb_tmp,
                                                  camera_center[:3], cam2world_transform]
                else:
                    print('0 faces were detected')
                    return False

                if discard_no_rgb:
                    colored_points = rgb[:, 0] > 0
                    xyz = xyz[colored_points]
                    rgb = rgb[colored_points]

                save_ply_in_file(output_path, xyz, rgb, cam2world_transform)

                print('Saved ply with color face mask %s' % output_path)
                if xyz_center_face is not None:
                    ply_path = output_path
                    extract_face_in_radius_from_ply(ply_path, xyz_center_face, path, suffix, objects_folder
                                                    , radius_cut=0.5)
                    print('Saved ply with ply face mask %s' % output_path)

        else:
            print('Transform not found for timestamp %s' % timestamp)
    return True


def extract_face_in_radius_from_ply(ply_path, xyz_center_face, path1, suffix, objects_folder=None, radius_cut=0.5):
    # Read point cloud from PLY
    pcd1_before_cutting = o3d.io.read_point_cloud(ply_path)
    # visualization before face mask
    # o3d.visualization.draw_geometries([pcd1])
    pcd1 = copy.deepcopy(pcd1_before_cutting)

    points = np.asarray(pcd1.points)
    colors = np.asarray(pcd1.colors)

    # helper for delete too far points
    # Sphere center and radius
    center = np.array(xyz_center_face)

    # Calculate distances to center, set new points
    distances = np.linalg.norm(points - center, axis=1)
    mask_dist = distances <= radius_cut
    filterd_points = points[mask_dist]
    filterd_colors = colors[mask_dist]
    pcd1.points = o3d.utility.Vector3dVector(filterd_points)
    pcd1.colors = o3d.utility.Vector3dVector(filterd_colors)

    # visualization after face
    # o3d.visualization.draw_geometries([pcd1])

    # Write point cloud out
    output_path = os.path.join(objects_folder, 'only_face.ply')
    # output_path = str(path1)[:-4] + f'{suffix}_only_face.ply'
    o3d.io.write_point_cloud(output_path, pcd1)


def save_ply_in_file(output_path, points, rgb=None, cam2world_transform=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(rgb)
    # pcd.estimate_normals()# our change
    if cam2world_transform is not None:
        # Camera center
        camera_center = (cam2world_transform) @ np.array([0, 0, 0, 1])
        # o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_center[:3])

    o3d.io.write_point_cloud(output_path, pcd)


def load_extrinsics(extrinsics_path):
    assert Path(extrinsics_path).exists()
    mtx = np.loadtxt(str(extrinsics_path), delimiter=',').reshape((4, 4))
    return mtx


def get_points_in_cam_space(img, lut):
    img = np.tile(img.flatten().reshape((-1, 1)), (1, 3))
    points = img * lut
    remove_ids = np.where(np.sum(points, axis=1) < 1e-6)[0]
    points = np.delete(points, remove_ids, axis=0)
    points /= 1000.
    return points


def cam2world(points, rig2cam, rig2world):
    homog_points = np.hstack((points, np.ones((points.shape[0], 1))))
    cam2world_transform = rig2world @ np.linalg.inv(rig2cam)
    world_points = cam2world_transform @ homog_points.T
    return world_points.T[:, :3], cam2world_transform


def extract_timestamp(path):
    return float(path.split('.')[0])


def load_rig2world_transforms(path):
    transforms = {}
    data = np.loadtxt(str(path), delimiter=',')
    value = data
    # for value in data:
    #     timestamp = value[0]
    #     transform = value[1:].reshape((4, 4))
    #     transforms[timestamp] = transform
    timestamp = value[0]
    transform = value[1:].reshape((4, 4))
    transforms[timestamp] = transform
    return transforms


def save_nparr_in_txtfile(nparr, folder, name):
    # a_file = open(str(folder / 'lut_arr.txt'), "w")
    with open(str(folder / str(name + '.txt')), 'w') as a_file:
        for row in nparr:
            np.savetxt(a_file, row)
    # a_file.close()


def load_lut_from_nptxt(folder):
    Depth_Long_Throw_dir_path = os.path.join(folder, 'Depth Long Throw')
    depth_img_lst = os.listdir(Depth_Long_Throw_dir_path)
    for x_file in depth_img_lst:
        if x_file.endswith(".pgm"):
            depth_img_path = os.path.join(Depth_Long_Throw_dir_path, x_file)
            break
    assert os.path.isfile(depth_img_path)
    # load depth img
    img = cv2.imread(str(depth_img_path), -1)
    height, width = img.shape
    lut = np.loadtxt(str(folder / 'lut_arr.txt'))
    lut = lut.reshape(height * width, 3)
    return lut


def save_ply_from_client(folder, stream_live=True, ext_mat_glob=None, lut_arr_glob=None, objects_folder=None
                         , radius_cut=0.5):
    save_in_cam_space = False

    discard_no_rgb = True
    clamp_min = 0.
    clamp_max = 0.
    depth_path_suffix = ''
    disable_project_pinhole = False

    print("")
    print("start Saving point clouds of face only")
    calib = r'{}_lut.bin'.format(sensor_name)
    extrinsics = r'{}_extrinsics.txt'.format(sensor_name)
    calib_path = folder / calib
    rig2campath = folder / extrinsics

    rig2world = r'{}_rig2world.txt'.format(sensor_name)
    rig2world_path = folder / rig2world if not save_in_cam_space else ''

    # check if we have pv
    # has_pv = False
    # try:
    #     if __name__ == '__main__':
    #         extract_tar_file(str(folder / 'PV.tar'), folder / 'PV')
    # except FileNotFoundError:
    #     pass

    pv_info_path = sorted(folder.glob(r'*pv.txt'))
    has_pv = len(list(pv_info_path)) > 0
    if has_pv:
        (pv_timestamps, focal_lengths, pv2world_transforms, ox,
         oy, _, _) = load_pv_data(list(pv_info_path)[0])
        principal_point = np.array([ox, oy])
    else:
        pv_timestamps = focal_lengths = pv2world_transforms = ox = oy = principal_point = None
    # lookup table to extract xyz from depth

    lut = load_lut(calib_path)
    # lut = load_lut_from_nptxt(folder)

    # from camera to rig space transformation (fixed)

    rig2cam = load_extrinsics(rig2campath)

    # from rig to world transformations (one per frame)
    rig2world_transforms = load_rig2world_transforms(
        rig2world_path) if rig2world_path != '' and Path(rig2world_path).exists() else None
    depth_path = Path(folder / sensor_name)
    depth_path.mkdir(exist_ok=True)
    is_dir2 = depth_path.is_dir()

    pinhole_folder = None
    if not disable_project_pinhole and has_pv:
        # Create folders for pinhole projection
        pinhole_folder = folder / 'pinhole_projection'
        pinhole_folder.mkdir(exist_ok=True)

        pinhole_folder_rgb = pinhole_folder / 'rgb'
        pinhole_folder_rgb.mkdir(exist_ok=True)

        pinhole_folder_depth = pinhole_folder / 'depth'
        pinhole_folder_depth.mkdir(exist_ok=True)

    # Depth path suffix used for now only if we load masked AHAT
    depth_paths = sorted(depth_path.glob('*[0-9]{}.pgm'.format(depth_path_suffix)))
    assert len(list(depth_paths)) > 0
    face_detected = False
    # Create shared dictionary to save odometry and file list
    manager = multiprocessing.Manager()
    shared_dict = manager.dict()
    multiprocess_pool = multiprocessing.Pool(multiprocessing.cpu_count())
    for path in depth_paths:
        face_detected = save_single_pcloud(shared_dict,
                                           path,
                                           folder,
                                           save_in_cam_space,
                                           lut,
                                           has_pv,
                                           focal_lengths,
                                           principal_point,
                                           rig2world_transforms,
                                           rig2cam,
                                           pv_timestamps,
                                           pv2world_transforms,
                                           discard_no_rgb,
                                           clamp_min,
                                           clamp_max,
                                           depth_path_suffix,
                                           disable_project_pinhole,
                                           objects_folder=objects_folder,
                                           radius_cut=0.5
                                           )
    multiprocess_pool.close()
    multiprocess_pool.join()
    if face_detected:
        if not disable_project_pinhole and has_pv and face_detected:
            save_output_txt_files(pinhole_folder, shared_dict)
        print('Done saving')
        return True
    return False


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process recorded data.')
    parser.add_argument("--recording_path", required=True,
                        help="Path to recording folder")
    parser.add_argument("--project_hand_eye",
                        required=False,
                        action='store_true',
                        help="Project hand joints (and eye gaze, if recorded) to rgb images")

    args = parser.parse_args()

    folder = w_path = Path(args.recording_path)
    is_dir = folder.is_dir()

    save_ply_from_client(folder, stream_live=False, radius_cut=0.5)
