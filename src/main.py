from supervisor import Supervisor
import numpy as np


def main():
    config = {
        "urdf_path": "../robot.urdf",
        "end_effector_frame_name": "Tip_Gripper_Servo",
        "use_meshcat": True,

        "udp_ip": "0.0.0.0",
        "udp_port": 5005,
        "udp_timeout_s": 0.20,
        "hand_key": "hand_right",

        "loop_dt": 0.03,
        "idle_sleep": 0.01,
        "print_debug": True,

        "filter_alpha": 0.18,
        "max_missing_frames": 8,
        "max_jump": 0.35,

        "trajectory_max_step": 0.03,

        "arduino_enabled": False,
        "arduino_port": "/dev/ttyUSB0",
        "arduino_baudrate": 115200,

        # Bezpieczniejsze mapowanie startowe.
        # x_robot: głównie od z kamery
        # y_robot: głównie od x kamery
        # z_robot: głównie od -y kamery
        "camera_to_robot_transform": np.array([
    [0.0,   0.0,  0.45,  0.05],
    [-0.90, 0.0,  0.0,   0.00],
    [0.0,  0.45, 0.0,   0.28],
    [0.0,   0.0,  0.0,   1.00],
], dtype=float),

"workspace_limits": np.array([
    [0.18, 0.65],
    [-0.25, 0.25],
    [0.18, 0.55],
], dtype=float),
    }

    app = Supervisor(config)
    app.run()


if __name__ == "__main__":
    main()