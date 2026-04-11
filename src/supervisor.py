import time
import numpy as np

from drake_sim import DrakeArmController
from kinect import KinectHandTracker
from mapping import CoordinateMapper
from pose import HandPoseFilter
from trajectory import TrajectorySmoother
from serial_bridge import ArduinoBridge


class Supervisor:
    def __init__(self, config: dict):
        self.config = config

        self.controller = DrakeArmController(
            urdf_path=config["urdf_path"],
            end_effector_frame_name=config["end_effector_frame_name"],
            use_meshcat=config.get("use_meshcat", True),
        )

        self.hand_tracker = KinectHandTracker(
            udp_ip=config.get("udp_ip", "0.0.0.0"),
            udp_port=config.get("udp_port", 5005),
            timeout_s=config.get("udp_timeout_s", 0.2),
            hand_key=config.get("hand_key", "hand_right"),
        )

        self.mapper = CoordinateMapper(
            transform_4x4=config["camera_to_robot_transform"],
            workspace_limits=config["workspace_limits"],
        )

        self.hand_filter = HandPoseFilter(
            alpha=config.get("filter_alpha", 0.18),
            max_missing_frames=config.get("max_missing_frames", 8),
            max_jump=config.get("max_jump", 0.35),
        )

        self.smoother = TrajectorySmoother(
            max_step=config.get("trajectory_max_step", 0.03)
        )

        self.arduino = ArduinoBridge(
            port=config.get("arduino_port", "/dev/ttyUSB0"),
            baudrate=config.get("arduino_baudrate", 115200),
            enabled=config.get("arduino_enabled", False),
        )



        self.loop_dt = config.get("loop_dt", 0.03)
        self.idle_sleep = config.get("idle_sleep", 0.01)
        self.print_debug = config.get("print_debug", True)

        self.last_valid_q = self.controller.q_home.copy()
        self.last_hand_state = None
        self.last_hand_confidence = None
        self.last_print_t = 0.0

    def run(self) -> None:
        self.hand_tracker.start()
        self.arduino.connect()

        print("Supervisor started")

        try:
            while True:
                measurement = self.hand_tracker.get_hand_measurement()

                if measurement is None:
                    self.controller.update_visualization(self.last_valid_q)
                    time.sleep(self.idle_sleep)
                    continue

                raw_hand = measurement["position"]
                hand_state = measurement["hand_state"]
                hand_confidence = measurement["hand_confidence"]

                filtered_hand = self.hand_filter.update(raw_hand)

                if filtered_hand is None:
                    self.controller.update_visualization(self.last_valid_q)
                    time.sleep(self.idle_sleep)
                    continue

                target_robot = self.mapper.camera_to_robot(filtered_hand)
                smooth_target = self.smoother.update(target_robot)


                q = self.controller.solve_ik(smooth_target)

                if q is not None:
                    gripper_closed = self.controller.hand_state_to_gripper_closed(
                        hand_state,
                        hand_confidence
                    )
                    q = self.controller.set_gripper_closed(q, gripper_closed)

                    self.last_valid_q = q
                    self.last_hand_state = hand_state
                    self.last_hand_confidence = hand_confidence

                    self.controller.update_visualization(q)

                    angles_deg = self.controller.get_joint_angles_deg(q)
                    self.arduino.send_joint_angles_deg(angles_deg)

                    if self.print_debug:
                        now = time.time()
                        if now - self.last_print_t > 0.4:
                            print("raw_hand:", np.round(raw_hand, 3))
                            print("filtered_hand:", np.round(filtered_hand, 3))
                            print("target_robot:", np.round(target_robot, 3))
                            print("smooth_target:", np.round(smooth_target, 3))
                            print("hand_state:", hand_state, "confidence:", hand_confidence)
                            print("gripper_closed:", gripper_closed)
                            print("q:", np.round(q, 3))
                            self.last_print_t = now
                else:
                    self.controller.update_visualization(self.last_valid_q)

                time.sleep(self.loop_dt)

        except KeyboardInterrupt:
            print("Supervisor stopped by user")
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self.hand_tracker.stop()
        self.arduino.close()
