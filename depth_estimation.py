import cv2
import numpy as np

class DETECTION():
    """
    Detects objects in the image.
    """
    def __init__(self):
        pass

    def detect(self, camera_type, image, height, width):
        """
        Detects objects in the image.
        """
        rgb   = np.reshape(image[2], (height, width, 4))[:, :, :3]
        rgb   = np.ascontiguousarray(rgb).astype(np.uint8)

        hsv  = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))   # Red hue

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M["m00"] > 0:
                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])

            # ---- Visual feedback ----
            cv2.circle(rgb, (u, v), 6, (0, 255, 0), 2)
            cv2.putText(rgb, f"Cube detected", (u+8, v-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            if camera_type == "overhead":
                return True, u, v, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif camera_type == "wrist":
                return u, v
        else:
            if camera_type == "overhead":
                return False, None, None, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif camera_type == "wrist":
                return None, None

    def bgr_to_rgb(self, image, height, width):
        """
        Converts BGR image to RGB image.
        """
        rgb   = np.reshape(image[2], (height, width, 4))[:, :, :3]
        rgb   = np.ascontiguousarray(rgb).astype(np.uint8)

        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def pixel_to_depth(self, image, height, width, u, v, fx, fy, cx, cy, far):
        """
        Convert pixel coordinates to depth in meters.
        :param image: Image data from PyBullet
        :param height: Image height
        :param width: Image width
        :param u: X coordinate of the pixel
        :param v: Y coordinate of the pixel
        :param fx: Focal length in X
        :param fy: Focal length in Y
        :param cx: Principal point in X
        :param cy: Principal point in Y
        :param far: Far clipping plane distance
        :return: Depth in meters
        """
        depth_buffer = np.reshape(image[3], (height, width))
        near = 0.01

        # z = depth_buffer[v, u]

        # Z = (far * near) / (far - (far - near) * depth_buffer)
        z_buffer = depth_buffer[v, u]
        Z = (2.0 * near * far) / (far + near - (2.0 * z_buffer - 1.0) * (far - near))

        return Z

    def compute_intrinsics(self, width, height, fov):
        """
        Compute the intrinsic matrix of the camera.
        :param width: Image width
        :param height: Image height
        :param fov: Field of view
        :return: Intrinsic matrix
        """
        aspect = width / height
        fov_rad = np.deg2rad(fov)

        fy = height / (2 * np.tan(fov_rad/2))
        fx = fy

        cx = width / 2
        cy = height / 2

        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

        return K, fx, fy, cx, cy

    def projection_point(self, u, v, Z, fx, fy, cx, cy):
        """
        Convert pixel coordinates to 3D point in camera frame.
        :param u: X coordinate of the pixel
        :param v: Y coordinate of the pixel
        :param Z: Depth in meters
        :param fx: Focal length in X
        :param fy: Focal length in Y
        :param cx: Principal point in X
        :param cy: Principal point in Y
        :return: 4D homogeneous coordinate of the point in camera frame
        """
        Xc = (u - cx) * Z / fx
        Yc = -(v - cy) * Z / fy
        Zc = Z

        return np.array([Xc, Yc, Zc, 1.0])

    def projection_world(self, xyz_cam, T_world_cam):
        """
        Convert 3D point in camera frame to 3D point in world frame.
        :param xyz_cam: 3D point in camera frame
        :param T_world_cam: Transformation matrix from world to camera frame
        :return: 3D point in world frame
        """
        point_cam = xyz_cam.reshape(4,1)
        point_world = T_world_cam @ point_cam

        return point_world[:3,0].astype(float)

    def compute_y_axis(self, v, height):
        """
        Compute the Y axis of the robot.
        :param v: Y coordinate of the pixel
        :param height: Image height
        :return: Y axis
        """
        y_value = height // 2
        if v < y_value:
            return -1
        else:
            return 1
