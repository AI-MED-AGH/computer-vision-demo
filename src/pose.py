from typing import Optional, Union
import numpy as np
from math import sqrt
#import testing   

MAX_MISSING_FRAMES = 5
ALPHA = 0.05
MAX_JUMP = 15

class HandPoseFilter:


    def __init__(self, alpha: float = ALPHA, max_missing_frames: int = MAX_MISSING_FRAMES, 
                 max_jump: float = MAX_JUMP):
        
        '''
        self.alpha: float = 0.25 , tells us how important the new point is.
        self.filtered: np.array, the current filtered point.
        self.missing_count: int, counts how many consecutive frames have been missing.
        self.max_missing_frames: int, the maximum number of consecutive missing frames 
        before we reset the filter.
        self.max_jump: float, the maximum allowed euclidean distance between the new point and the
        current filtered point
        '''

        if alpha < 0 or alpha > 1:
            raise ValueError("Alpha must be between 0 and 1")
        
        if max_missing_frames < 0:
            raise ValueError("max_missing_frames must be non-negative")
        
        if max_jump < 0:
            raise ValueError("max_jump must be non-negative")

        self.alpha = alpha
        self.filtered: Optional[np.ndarray] = None               # last saved value
        self.missing_count = 0
        self.max_missing_frames = max_missing_frames
        self.max_jump = max_jump
       

    def update(self, point: np.array) -> np.ndarray:

        '''
        based on EMA filter
        returns np.array object if point exists and point is not an outlier
        returns None in other cases
        '''
        
        if point is None or np.isnan(point).any():
            self.missing_count +=1
            return self.filtered
        
        if len(point) != 3:
            raise ValueError("Point must be a 3D coordinate (x, y, z)")
        
        if self.filtered is None or self.missing_count >= self.max_missing_frames:
            self.filtered = point

        elif self._is_outlier(point):
            self.missing_count += 1
            return self.filtered
        
        else:
            ema = self.alpha * point + (1 - self.alpha) * self.filtered
            self.filtered = ema 
        
        
        self.missing_count = 0
        return self.filtered                    

        

    def _is_outlier(self, point: np.array) -> bool:

        '''
        Method checks if given point is an outlier
        if euclidean distance is greater than defined
        max_jump -> outlier

        return_type: bool
        '''
        
        if self.filtered is None:
            return False       
                        
        
        x1, x2 = (point[0], self.filtered[0])
        y1, y2 = (point[1], self.filtered[1])
        z1, z2 = (point[2], self.filtered[2])

        euclidean_distance_3d = sqrt( (x1 - x2)**2 + (y1 - y2) **2 + (z1 - z2)**2)

        if euclidean_distance_3d > self.max_jump:
            return True                                # is outlier

        return False                                   # is not outlier
        


#testing.test() for local testing
