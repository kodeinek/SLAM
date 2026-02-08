import sys
import signal
import atexit
import time
import numpy as np
import cv2
import pyzed.sl as sl



class Camera:
    def __init__(self, min_dist=0.3, max_dist=5.0):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.image_left = sl.Mat()
        self.pc = sl.Mat()
        self.pose = sl.Pose()
        self.normals = sl.Mat()
        self.min_dist = float(min_dist)
        self.max_dist = float(max_dist)

        self.last_timestamp = None
        self.runtime_parameters = sl.RuntimeParameters()
        self.K = None



    def open_from_svo(self, svo_path):
        self.init_params.set_from_svo_file(svo_path)
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
        self.init_params.set_from_svo_file(svo_path)
        self.init_params.depth_maximum_distance = self.max_dist
        self.init_params.svo_real_time_mode = False
  

        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            return False

        tracking = sl.PositionalTrackingParameters()
        tracking.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
        tracking.enable_area_memory = True
        tracking.enable_pose_smoothing = False
        tracking.enable_imu_fusion = False
        tracking.set_floor_as_origin = False

        
        self.runtime_parameters.confidence_threshold = 60
        self.runtime_parameters.enable_fill_mode = True


        cam_params = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
        self.K = np.array(
            [
                [cam_params.fx, 0.0, cam_params.cx],
                [0.0, cam_params.fy, cam_params.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )



        err = self.zed.enable_positional_tracking(tracking)
        if err != sl.ERROR_CODE.SUCCESS:
            self.close()
            return False

        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)
        atexit.register(self.close)
        return True

    def open_from_live(self,ip, port):

        self.init_params.set_from_stream(ip, port)
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_maximum_distance = self.max_dist
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS



        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            return False

        tracking = sl.PositionalTrackingParameters()
        tracking.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
        tracking.enable_area_memory = True
        tracking.enable_pose_smoothing = False
        tracking.enable_imu_fusion = True
        tracking.set_floor_as_origin = False

        self.runtime_parameters.confidence_threshold =60


        cam_params = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
        self.K = np.array(
            [
                [cam_params.fx, 0.0, cam_params.cx],
                [0.0, cam_params.fy, cam_params.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        err = self.zed.enable_positional_tracking(tracking)
        if err != sl.ERROR_CODE.SUCCESS:
            self.close()
            return False

        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)
        atexit.register(self.close)
        return True

    
    def close(self):
        try:
            self.zed.close()
        except Exception:
            pass

    def cleanup(self, signum=None, frame=None):
        self.close()
        sys.exit(0)

    def grab(self):
        if self.zed.grab(self.runtime_parameters) != sl.ERROR_CODE.SUCCESS:
            return None, None

        now = time.time()
        dt = 0.0 if self.last_timestamp is None else (now - self.last_timestamp)
        self.last_timestamp = now
        return dt, now

    def get_left_image(self):
        self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT_GRAY)
        return self.image_left.get_data()

    def get_pose_world(self, threshold=70):
        self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        t = np.array(self.pose.get_translation().get(), dtype=np.float64).reshape(3, 1)
        R = np.array(self.pose.get_rotation_matrix().r, dtype=np.float64)
        conf = int(self.pose.pose_confidence)
        ok = (conf > threshold) or (conf < 1)
        return R,t, ok

    def get_pose_camera(self):
        self.zed.get_position(self.pose, sl.REFERENCE_FRAME.CAMERA)
        t = np.array(self.pose.get_translation().get(), dtype=np.float64).reshape(3, 1)
        R = np.array(self.pose.get_rotation_matrix().r, dtype=np.float64)

        R = R.T
        t = -R @ t

        return R,t

    def get_normals(self):
        self.zed.retrieve_measure(self.normals, sl.MEASURE.NORMALS)
        return self.normals.get_data()[:, :, :3]

    def get_point_cloud(self):
        self.zed.retrieve_measure(self.pc, sl.MEASURE.XYZ)
        return self.pc.get_data()[:, :, :3]
    def retrieve_measurements(self):
        left_img = self.get_left_image()
        full_pc = self.get_point_cloud()
        normals = self.get_normals()
        return left_img, full_pc, normals


