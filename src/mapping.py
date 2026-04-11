import numpy as np


class CoordinateMapper:
    def __init__(self, transform_4x4: np.ndarray, workspace_limits: np.ndarray):
        self.transform_4x4 = np.asarray(transform_4x4, dtype=float).reshape(4, 4)
        self.workspace_limits = np.asarray(workspace_limits, dtype=float).reshape(3, 2)

    def camera_to_robot(self, point_camera: np.ndarray) -> np.ndarray:
        point_camera = np.asarray(point_camera, dtype=float).reshape(3)
        point_cam_to_trans = np.append(point_camera, 1.0)
        transformed_point = self.transform_4x4 @ point_cam_to_trans
        return self._clamp(transformed_point[:3])

    def _clamp(self, point_robot: np.ndarray) -> np.ndarray:
        point_robot = np.asarray(point_robot, dtype=float).reshape(3)
        limited_point = point_robot.copy()

        for i in range(3):
            left_lim, right_lim = self.workspace_limits[i]
            limited_point[i] = np.clip(point_robot[i], left_lim, right_lim)

        return limited_point