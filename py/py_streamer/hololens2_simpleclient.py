import os
import shutil
import socket
import struct
import abc
import threading
from datetime import datetime, timedelta
from collections import namedtuple, deque
from enum import Enum
import numpy as np
import cv2
import time
import argparse
from pathlib import Path
from StreamRecorderConverter.savePcloud_v2 import save_data_for_registration, save_ply_from_client, \
    delete_txt_from_dir_content
from StreamRecorderConverter.fast_registration import do_registration
from file_converter_obj_to_ply.convert import PLYwithoutRGB
from py_streamer.test_tcp_client import send_file_to_HL
from py_streamer.zmq_server import ServerZMQ

np.warnings.filterwarnings('ignore')
# check commit1
# Definitions
# Protocol Header Format
# see https://docs.python.org/2/library/struct.html#format-characters
VIDEO_STREAM_HEADER_FORMAT = "@qIIII20f"

VIDEO_FRAME_STREAM_HEADER = namedtuple(
    'SensorFrameStreamHeader',
    'Timestamp ImageWidth ImageHeight PixelStride RowStride fx fy principalPoint_x principalPoint_y '
    'PVtoWorldtransformM11 PVtoWorldtransformM12 PVtoWorldtransformM13 PVtoWorldtransformM14 '
    'PVtoWorldtransformM21 PVtoWorldtransformM22 PVtoWorldtransformM23 PVtoWorldtransformM24 '
    'PVtoWorldtransformM31 PVtoWorldtransformM32 PVtoWorldtransformM33 PVtoWorldtransformM34 '
    'PVtoWorldtransformM41 PVtoWorldtransformM42 PVtoWorldtransformM43 PVtoWorldtransformM44 '
)

RM_STREAM_HEADER_FORMAT = "@qIIII16f"

RM_FRAME_STREAM_HEADER = namedtuple(
    'SensorFrameStreamHeader',
    'Timestamp ImageWidth ImageHeight PixelStride RowStride '
    'rig2worldTransformM11 rig2worldTransformM12 rig2worldTransformM13 rig2worldTransformM14 '
    'rig2worldTransformM21 rig2worldTransformM22 rig2worldTransformM23 rig2worldTransformM24 '
    'rig2worldTransformM31 rig2worldTransformM32 rig2worldTransformM33 rig2worldTransformM34 '
    'rig2worldTransformM41 rig2worldTransformM42 rig2worldTransformM43 rig2worldTransformM44 '
)

EXT_LUT_STREAM_HEADER_FORMAT = "@II16f"

EXT_LUT_STREAM_HEADER = namedtuple(
    'EXTLUTStreamHeader',
    'ImageWidth ImageHeight '
    'extM11 extM12 extM13 extM14 '
    'extM21 extM22 extM23 extM24 '
    'extM31 extM32 extM33 extM34 '
    'extM41 extM42 extM43 extM44 '
)

# Each port corresponds to a single stream type
VIDEO_STREAM_PORT = 23940
AHAT_STREAM_PORT = 23941
EXT_LUT_STREAM_PORT = 23941
LOADING_PORT = 12345
# HOST = '192.168.0.80'
HOST = '10.0.0.1'
LOCAL_HOST = '127.0.0.1'
obj_to_ply_flag = False

HundredsOfNsToMilliseconds = 1e-4
MillisecondsToSeconds = 1e-3

thread_running = True
latest_header_video = None
latest_frame_video = None
latest_header_depth = None
latest_frame_depth = None

registration_was_pressed = True

ext_mat_glob = None
lut_arr_glob = None

class SensorType(Enum):
    VIDEO = 1
    AHAT = 2
    LONG_THROW_DEPTH = 3
    LF_VLC = 4
    RF_VLC = 5


class FrameReceiverThread(threading.Thread):
    def __init__(self, host, port, header_format, header_data):
        super(FrameReceiverThread, self).__init__()
        self.header_size = struct.calcsize(header_format)
        self.header_format = header_format
        self.header_data = header_data
        self.host = host
        self.port = port
        self.latest_frame = None
        self.latest_header = None
        self.socket = None

    def get_data_from_socket(self, from_camera='video'):
        # if from_camera == 'ahat':
        #     #should get extrinics and lut
        # read the header in chunks
        reply = self.recvall(self.header_size)

        if not reply:
            print('ERROR: Failed to receive data from stream.')
            return

        data = struct.unpack(self.header_format, reply)
        header = self.header_data(*data)

        # read the image in chunks
        row_stride = header.RowStride
        # if from_camera == 'ahat': #Long Throw
        #    row_stride = 2* row_stride
        image_size_bytes = header.ImageHeight * row_stride

        image_data = self.recvall(image_size_bytes)

        # header = None
        return header, image_data

    def recvall(self, size):
        msg = bytes()
        while len(msg) < size:

            part = self.socket.recv(size - len(msg))
            if part == '':
                break  # the connection is closed
            msg += part
        return msg

    def start_socket(self, name=''):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        # send_message(self.socket, b'socket connected at ')
        print('INFO: Socket', name, ' connected to ' + self.host + ' on port ' + str(self.port))

    def end_socket(self, name=''):
        self.socket.close()
        print('INFO:', name, ' Socket end')

    def start_listen(self):
        t = threading.Thread(target=self.listen)
        t.start()
        return t

    @abc.abstractmethod
    def listen(self):
        return

    @abc.abstractmethod
    def get_mat_from_header(self, header):
        return


class VideoReceiverThread(FrameReceiverThread):
    def __init__(self, host):
        super().__init__(host, VIDEO_STREAM_PORT, VIDEO_STREAM_HEADER_FORMAT,
                         VIDEO_FRAME_STREAM_HEADER)

    def listen(self):
        global thread_running
        thread_running = True
        global latest_header_video
        global latest_frame_video
        while thread_running:
            latest_header_video, image_data = self.get_data_from_socket()
            latest_frame_video = np.frombuffer(image_data, dtype=np.uint8).reshape((latest_header_video.ImageHeight,
                                                                                    latest_header_video.ImageWidth,
                                                                                    latest_header_video.PixelStride))

            # self.latest_frame = np.frombuffer(image_data, dtype=np.uint8).reshape((360,
            #                                                                        640,
            #                                                                        3))

    def get_mat_from_header(self, header):
        pv_to_world_transform = np.array(header[7:24]).reshape((4, 4)).T
        return pv_to_world_transform


class AhatReceiverThread(FrameReceiverThread):
    def __init__(self, host, ahat_stream_port, rm_stream_header_format, rm_frame_stream_header):
        super().__init__(host,
                         ahat_stream_port, rm_stream_header_format, rm_frame_stream_header)

    def listen(self):
        global thread_running
        thread_running = True
        global latest_header_depth
        global latest_frame_depth
        while thread_running:
            latest_header_depth, image_data = self.get_data_from_socket('ahat')
            latest_frame_depth = np.frombuffer(image_data, dtype=np.uint16)
            latest_frame_depth = np.reshape(latest_frame_depth,
                                            (latest_header_depth.ImageHeight, latest_header_depth.ImageWidth))
            pass
            # todo: maybe need to be uint8

    def get_mat_from_header(self, header):
        rig_to_world_transform = np.array(header[5:22]).reshape((4, 4)).T
        return rig_to_world_transform


class ExtLutReceiverThread(AhatReceiverThread):
    def get_data_from_socket(self, from_camera='video'):
        # if from_camera == 'ahat':
        #     #should get extrinics and lut
        # read the header in chunks
        reply = self.recvall(self.header_size)  # header is extrinsic

        if not reply:
            print('ERROR: Failed to receive data from stream.')
            return

        data = struct.unpack(self.header_format, reply)
        header = self.header_data(*data)
        ext_mat = self.get_mat_from_header(header)

        # read the lut in chunks
        lut_format = "@" + str(header.ImageHeight * header.ImageWidth * 3) + "f"
        lut_size_bytes = struct.calcsize(lut_format)
        lut_reply = self.recvall(lut_size_bytes)
        lut_arr = np.frombuffer(lut_reply, dtype="f")
        lut_arr = np.reshape(lut_arr, (-1, 3))

        # header = None
        return ext_mat, lut_arr

    def listen(self):
        global ext_mat_glob
        global lut_arr_glob
        ext_mat_glob, lut_arr_glob = self.get_data_from_socket('extlut')

    def get_mat_from_header(self, header):
        ext_mat = np.array(header[2:19]).reshape((4, 4)).T
        return ext_mat


def start_socket_and_listen(num_round=0):
    video_receiver = VideoReceiverThread(HOST)
    video_receiver.start_socket('video_' + str(num_round))

    ahat_receiver = AhatReceiverThread(HOST, AHAT_STREAM_PORT, RM_STREAM_HEADER_FORMAT, RM_FRAME_STREAM_HEADER)
    ahat_receiver.start_socket('ahat_' + str(num_round))

    t_video = video_receiver.start_listen()
    t_ahaht = ahat_receiver.start_listen()
    return video_receiver, ahat_receiver, t_video, t_ahaht




if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process recorded data.')
    parser.add_argument("--only_smaple", required=False, default=False,
                        help="sample once pcd")
    args = parser.parse_args()
    only_sample = args.only_smaple
    print('script start:')
    # print('getting extrincs and lut:')
    #
    # ext_lut_receiver = ExtLutReceiverThread(CLIENT,
    #                                         EXT_LUT_STREAM_PORT, EXT_LUT_STREAM_HEADER_FORMAT, EXT_LUT_STREAM_HEADER)
    # ext_lut_receiver.start_socket('ExtLut')
    # t_ext_lut = ext_lut_receiver.start_listen()
    #
    # # kill lut recv
    # time.sleep(2)
    # # get ext and lut
    # t_ext_lut.join()
    # ext_lut_receiver.end_socket('ExtLut')
    # print('ExtLut succeeded')

    file_path = Path(__file__)
    converter_folder = file_path.parent.parent
    reg_folder = Path(os.path.join(converter_folder, 'register_folder'))
    outputs_folder = Path(os.path.join(converter_folder, 'outputs'))
    ply_folder = Path(os.path.join(outputs_folder, 'ply_files'))
    obj_folder = Path(os.path.join(outputs_folder, 'obj_files'))
    txt_folder = Path(os.path.join(outputs_folder, 'txt_files'))

    is_dir = converter_folder.is_dir()

    # ZMQ server
    zmq_server = ServerZMQ(port=LOADING_PORT)
    zmq_server.init_server()
    rounds = 1
    video_receiver, ahat_receiver, t_video, t_ahaht = start_socket_and_listen(rounds)

    # get initialized ply from ct scan, must send the corresponding obj file
    ct_scan_path = os.path.join(ply_folder, 'face_oren1.ply')
    ct_scan_mesh_path = os.path.join(txt_folder, 'face_oren1_mesh.obj')
    while True:
        time.sleep(1)
        # kill
        thread_running = False
        t_video.join()
        t_ahaht.join()
        if registration_was_pressed:
            if latest_frame_depth is None:
                print('There is no depth frame!!')
                exit(1)
            if latest_frame_video is not None:  # and np.any(latest_frame_depth):
                # rgb header:
                # pTimestamp
                # outImageWidth
                # outImageHeight
                # image_width_intri, comment
                # image_height_intri, comment
                # outPixelStride
                # outRowStride
                # fx
                # fy
                # principalPoint_x
                # principalPoint_y
                # matrix 4X4- PVtoWorldtransform
                # rgb image - imageBufferAsVector

                video_reciver_header = latest_header_video
                video_reciver_frame = latest_frame_video
                cv2.imshow('Photo Video Camera Stream', video_reciver_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                # depth header:
                # absoluteTimestamp
                # imageWidth
                # imageHeight
                # pixelStride
                # rowStride
                # matrix 4*4 - rig2worldTransform
                # depth img - depthByteData

                ahat_receiver_header = None
                ahat_receiver_frame = None
                if latest_frame_depth is not None:
                    ahat_receiver_header = latest_header_depth
                    ahat_receiver_frame = latest_frame_depth
                    cv2.imshow('Depth Camera Stream', ahat_receiver_frame)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                # saving all data in the correct format
                save_data_for_registration(video_reciver_header, ahat_receiver_header,
                                           video_reciver_frame, ahat_receiver_frame,
                                           reg_folder)

                video_receiver.end_socket('video_' + str(rounds))
                ahat_receiver.end_socket('ahat_' + str(rounds))

                # rgb + depth to ply
                # save ply

                ply_was_saved = save_ply_from_client(reg_folder, objects_folder=ply_folder, radius_cut=0.5)

                if only_sample:
                    print('lets stop here for now, only sample, round num is ', rounds)
                    exit(0)
                transformed_obj_mesh_txt_path = None
                if ply_was_saved:
                    # get ply from streaming
                    streaming_face_path = os.path.join(ply_folder, 'only_face.ply')
                    saving_streaming_face_debug_path = os.path.join(ply_folder, 'only_face.ply')
                    now = datetime.now()
                    file_name = f'only_face_{now.strftime("%d_%m_%Y__%H_%M_%S")}.ply'
                    shutil.copyfile(streaming_face_path, os.path.join(ply_folder, file_name))
                    # delete all txt files
                    delete_txt_from_dir_content(txt_folder)
                    # get saving transformed obj as txt
                    # do registration and save results
                    print('start registration:')
                    print(f"source_path = {ct_scan_path}")
                    print(f"target_path = {streaming_face_path}")
                    print(f"source_mesh_path = {ct_scan_mesh_path}")

                    print(f"before reg, ct_scan_path={ct_scan_path}")
                    import open3d as o3d
                    source = o3d.io.read_point_cloud(ct_scan_path)
                    print(source)

                    print(f"before reg, streaming_face_path={streaming_face_path}")
                    source = o3d.io.read_point_cloud(streaming_face_path)
                    print(source)

                    transformed_obj_mesh_txt_path, transformed_obj_mesh_obj_path = do_registration(
                        source_path=ct_scan_path,
                        target_path=streaming_face_path,
                        source_mesh_path=ct_scan_mesh_path)
                    print('end registration.')
                    print(f"after reg, transformed_obj_mesh_obj_path={transformed_obj_mesh_obj_path}")
                    with open(transformed_obj_mesh_obj_path, "r") as f:
                        print(f"len of files is {len(f.readlines())}")
                    time.sleep(1)
                    # send registration back
                    print(f"sending file - {transformed_obj_mesh_txt_path}")
                    send_file_to_HL(transformed_obj_mesh_txt_path, zmq_server)

                    if obj_to_ply_flag:
                        # transformed_obj_mesh_path to ply
                        transformed_ply_file_path = os.path.join(ply_folder, f'ctScan_transformed_{now.strftime("%d_%m_%Y__%H_%M_%S")}.ply')
                        # PLYwithoutRGB(transformed_obj_mesh_obj_path, transformed_ply_file_path
                        #o3d convert
                        obj_o3d = o3d.io.read_triangle_mesh(transformed_obj_mesh_obj_path)
                        o3d.io.write_point_cloud(filename=transformed_ply_file_path,
                                                 pointcloud=o3d.cpu.pybind.geometry.PointCloud(obj_o3d.vertices))
                        # send source as ply(transformed_obj_mesh)
                        ct_scan_path = transformed_ply_file_path
                        ct_scan_mesh_path = transformed_obj_mesh_obj_path
                else:
                    print('ply was not saved,and therefore there was not registration')

                # start again
                rounds += 1

        video_receiver, ahat_receiver, t_video, t_ahaht = start_socket_and_listen(rounds)
