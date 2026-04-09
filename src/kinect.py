import time
import math
import numpy as np
import cv2
import mediapipe as mp

class KinectHandTracker:
    def __init__(self, use_mock: bool = True):
        self.use_mock = use_mock
        self.sim_t = 0.0
        self.device_handle = None
        self._is_running = False
        self._last_time = time.time()

        if not self.use_mock:
            self.mp_hands = mp.solutions.hands
            self.hands_detector = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            self.fx = 500.0
            self.fy = 500.0
            self.cx = 320.0
            self.cy = 240.0

    def start(self) -> None:
        self._is_running = True
        self._last_time = time.time()
        if not self.use_mock:
            self.device_handle = cv2.VideoCapture(cv2.CAP_OPENNI2)

    def stop(self) -> None:
        self._is_running = False
        if not self.use_mock:
            if self.device_handle is not None:
                self.device_handle.release()
                self.device_handle = None
            if hasattr(self, 'hands_detector'):
                self.hands_detector.close()

    def get_hand_position_3d(self):
        if not self._is_running:
            return None

        if self.use_mock:
            current_time = time.time()
            dt = current_time - self._last_time
            self._last_time = current_time
            self.sim_t += dt

            x = 0.3 * math.sin(self.sim_t * 1.2)
            y = 0.2 * math.cos(self.sim_t * 0.8)
            z = 1.0 + 0.15 * math.sin(self.sim_t * 0.5)

            return np.array([x, y, z], dtype=float)

        else:
            try:
                color_frame = self._get_color_frame()
                depth_frame = self._get_depth_frame()

                if color_frame is None or depth_frame is None:
                    return None

                hand_2d = self._detect_hand_2d(color_frame)
                if hand_2d is None:
                    return None

                u, v = hand_2d
                
                if v >= depth_frame.shape[0] or u >= depth_frame.shape[1]:
                    return None

                depth_mm = depth_frame[v, u] 

                if depth_mm <= 0:
                    return None

                depth_meters = float(depth_mm) / 1000.0

                return self._project_to_3d(u, v, depth_meters)

            except Exception:
                return None

    def _get_color_frame(self):
        if self.device_handle and self.device_handle.isOpened():
            self.device_handle.grab()
            ret, frame = self.device_handle.retrieve(cv2.CAP_OPENNI_BGR_IMAGE)
            if ret:
                return frame
        return None

    def _get_depth_frame(self):
        if self.device_handle and self.device_handle.isOpened():
            ret, depth_map = self.device_handle.retrieve(cv2.CAP_OPENNI_DEPTH_MAP)
            if ret:
                return depth_map
        return None

    def _detect_hand_2d(self, color_frame):
        rgb_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        
        results = self.hands_detector.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmark = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
            
            h, w, _ = color_frame.shape
            u = int(landmark.x * w)
            v = int(landmark.y * h)
            
            return (u, v)
            
        return None

    def _project_to_3d(self, u: int, v: int, depth_value: float):
        z = depth_value
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        return np.array([x, y, z], dtype=float)