from drake_sim import DrakeArmController
import numpy as np
import time

#test modelu chwytaka
class Supervisor:
    def __init__(self, config: dict):
        self.config = config
        self.controller = DrakeArmController(
            urdf_path=config["urdf_path"],
            end_effector_frame_name=config["end_effector_frame_name"],
            use_meshcat=config.get("use_meshcat", True),
        )

    def run(self) -> None:
        t = 0.0
        dt = 0.03

        while True:
            q = np.zeros(self.controller.num_positions)

            if self.controller.num_positions > 0:
                q[0] = 0.35 * np.sin(0.8 * t)
            if self.controller.num_positions > 1:
                q[1] = 0.25 * np.sin(1.0 * t + 0.4)
            if self.controller.num_positions > 2:
                q[2] = 0.20 * np.sin(1.2 * t + 0.8)
            if self.controller.num_positions > 3:
                q[3] = 0.18 * np.sin(0.9 * t + 1.1)
            if self.controller.num_positions > 4:
                q[4] = 0.15 * np.sin(1.1 * t + 1.5)

            # reszta, czyli chwytak, zostaje na 0
            self.controller.update_visualization(q)

            time.sleep(dt)
            t += dt