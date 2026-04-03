import numpy as np


class CoordinateMapper:
    def __init__(self, transform_4x4: np.ndarray, workspace_limits: np.ndarray):
        self.transform_4x4 = np.array(transform_4x4)
        self.workspace_limits = workspace_limits

    def camera_to_robot(self, point_camera: np.ndarray) -> np.ndarray:
        point_cam_to_trans = np.append(point_camera, 1.0)
        # Assumed standard in robotics column convention for matrix
        transformed_point = self.transform_4x4 @ point_cam_to_trans
        return self._clamp(transformed_point[:3])

    def _clamp(self, point_robot: np.ndarray) -> np.ndarray:
        limited_point = point_robot.copy()
        for i in range(3):
            left_lim, right_lim = self.workspace_limits[i]
            if point_robot[i] < left_lim:
                limited_point[i] = left_lim
            elif point_robot[i] > right_lim:
                limited_point[i] = right_lim
            else:
                limited_point[i] = point_robot[i]
        return np.array(limited_point)