import numpy as np


class TrajectorySmoother:
    def __init__(self, max_step: float = 0.02):
        self.max_step = max_step
        self.current_target = None

    def update(self, desired_target):
        desired_target = np.asarray(desired_target, dtype=float)

        if desired_target.shape != (3,):
            raise ValueError("desired_target musi mieć postać np.array([x, y, z])")

        if self.current_target is None:
            self.current_target = desired_target.copy()
            return self.current_target.copy()

        delta = desired_target - self.current_target
        distance = np.linalg.norm(delta)

        if distance <= self.max_step or distance == 0.0:
            self.current_target = desired_target.copy()
        else:
            direction = delta / distance
            self.current_target = self.current_target + direction * self.max_step

        return self.current_target.copy()