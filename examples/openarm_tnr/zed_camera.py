import numpy as np
import cv2
import pyzed.sl as sl

from oa_utils import resize_image


class ZEDMiniCamera:
    def __init__(self, resolution=sl.RESOLUTION.HD720, fps=30):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = resolution
        self.init_params.depth_mode = sl.DEPTH_MODE.NONE
        self.init_params.camera_fps = fps
        self.runtime_params = sl.RuntimeParameters()
        self.image_left  = sl.Mat()
        self.image_right = sl.Mat()
        self.height = None
        self.width  = None
        self.deisred_height = 800
        self.deisred_width  = 1280

    def get_roi(self):
        img_left, img_right = self.get_frames()
        if img_left is None or img_right is None:
            raise ValueError("Could not get frame from ZED Mini camera")

        img_resized = resize_image(img=img_left,  desired_size=(self.deisred_width, self.deisred_height))
        img_resized = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        img_R       = resize_image(img=img_right, desired_size=(self.deisred_width, self.deisred_height))
        img_R       = cv2.cvtColor(img_R, cv2.COLOR_BGR2RGB)

        print("\n[ROI] Draw a rectangle around the target object.")
        print("      Click and drag to select, then press SPACE or ENTER to confirm.")
        print("      Press C to cancel.\n")
        img_draw = cv2.cvtColor(img_resized.copy(), cv2.COLOR_BGR2RGB)
        x, y, w, h = cv2.selectROI("Template ROI Selection", img_draw,
                                    fromCenter=False, showCrosshair=True)
        box = [x, y, x + w, h + y]
        cv2.destroyAllWindows()
        return box, img_resized, img_R

    def open_cam(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open ZED Mini with error = {err}")
            return False
        print("ZED Mini Ready !")
        return True

    def get_frames(self):
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_left,  sl.VIEW.LEFT)
            self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
            left_cv  = cv2.cvtColor(self.image_left.get_data(),  cv2.COLOR_BGRA2BGR)
            right_cv = cv2.cvtColor(self.image_right.get_data(), cv2.COLOR_BGRA2BGR)
            return left_cv, right_cv
        return None, None

    def get_calibration(self):
        cam_info = self.zed.get_camera_information()
        cal = cam_info.camera_configuration.calibration_parameters

        def _K(p):
            K = np.zeros((3, 3)); K[2, 2] = 1.0
            K[0, 0] = p.fx; K[1, 1] = p.fy
            K[0, 2] = p.cx; K[1, 2] = p.cy
            return K

        L = _K(cal.left_cam)
        R = _K(cal.right_cam)
        camera_calib = {
            "cam0":     L.tolist(),
            "cam1":     R.tolist(),
            "doffs":    R[0, 2] - L[0, 2],
            "baseline": np.abs(cal.stereo_transform.get_translation().get()[0]),
            "width":    cal.left_cam.image_size.width,
            "height":   cal.left_cam.image_size.height,
            "ndisp":    250,
        }
        self.width  = cal.left_cam.image_size.width
        self.height = cal.left_cam.image_size.height
        return camera_calib

    def close_cam(self):
        self.zed.close()
        print("ZED Mini closed.")
