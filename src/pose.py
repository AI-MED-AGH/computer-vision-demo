from typing import Optional
import numpy as np

MAX_MISSING_FRAMES = 8
ALPHA = 0.18
MAX_JUMP = 0.35


class HandPoseFilter:
    def __init__(
        self,
        alpha: float = ALPHA,
        max_missing_frames: int = MAX_MISSING_FRAMES,
        max_jump: float = MAX_JUMP
    ):
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("Alpha must be between 0 and 1")

        if max_missing_frames < 0:
            raise ValueError("max_missing_frames must be non-negative")

        if max_jump < 0:
            raise ValueError("max_jump must be non-negative")

        self.alpha = alpha
        self.filtered: Optional[np.ndarray] = None
        self.missing_count = 0
        self.max_missing_frames = max_missing_frames
        self.max_jump = max_jump

    def update(self, point) -> Optional[np.ndarray]:
        if point is None:
            self.missing_count += 1
            if self.missing_count >= self.max_missing_frames:
                self.filtered = None
            return self.filtered

        point = np.asarray(point, dtype=float).reshape(3)

        if not np.isfinite(point).all():
            self.missing_count += 1
            if self.missing_count >= self.max_missing_frames:
                self.filtered = None
            return self.filtered

        if self.filtered is None or self.missing_count >= self.max_missing_frames:
            self.filtered = point.copy()
            self.missing_count = 0
            return self.filtered

        distance = np.linalg.norm(point - self.filtered)
        if distance > self.max_jump:
            self.missing_count += 1
            return self.filtered

        self.filtered = self.alpha * point + (1.0 - self.alpha) * self.filtered
        self.missing_count = 0
        return self.filtered