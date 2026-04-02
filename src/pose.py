import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

class HandPoseFilter:    

    '''
    self.alpha: float = 0.25 , tells us how important the new point is.
    self.filtered: np.array, the current filtered point.
    self.missing_count: int, counts how many consecutive frames have been missing.
    self.max_missing_frames: int, the maximum number of consecutive missing frames 
    before we reset the filter.
    self.max_jump: float, the maximum allowed eucliean distance between the new point and the
    current filtered point
    '''


    def __init__(self, alpha: float = 0.25, max_missing_frames: int = 10, 
                 max_jump: float = 30):
        
        self.alpha = alpha
        self.filtered = None                        # last saved value
        self.missing_count = 0
        self.max_missing_frames = max_missing_frames
        self.max_jump = max_jump
       

    def update(self, point: np.array) -> np.array | None:

        '''
        based on EMA filter
        returns np.array object if point exists and point is not an outlier
        returns None in other cases
        '''
        
        if point is None or np.isnan(point).any() or self._is_outlier(point):
            self.missing_count +=1
            return             
        
        if self.filtered is None or self.missing_count > self.max_missing_frames:
            self.filtered = point
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

        if len(point) != 3 or len(self.filtered) != 3:
            raise ValueError("Point must be a 3D coordinate (x, y, z)")
                        
        
        x1, x2 = (point[0], self.filtered[0])
        y1, y2 = (point[1], self.filtered[1])
        z1, z2 = (point[2], self.filtered[2])

        eucldean_distance_3d = sqrt( (x1 - x2)**2 + (y1 - y2) **2 + (z1 - z2)**2)

        if eucldean_distance_3d > self.max_jump:
            return True                                # is outlier

        return False                                   # is not outlier
        



n_frames = 300
t = np.arange(n_frames)

true_x = 1 * t
true_y = 50 + 0.2 * t
true_z = 20 + 1 * t

# szum
noise = np.random.normal(0, 10, (n_frames, 3))
raw_data_3d = np.column_stack((true_x, true_y, true_z)) + noise

# outlier + dziura
raw_data_3d[25] = np.array([25, 55, 200]) 
raw_data_3d[40:50] = np.nan               

# 3d rysowanie
fig = plt.figure(figsize=(12, 8))
# Tworzymy podwykres 3D
ax = fig.add_subplot(111, projection='3d')

x_coords = raw_data_3d[:, 0]
y_coords = raw_data_3d[:, 1]
z_coords = raw_data_3d[:, 2]

ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o', s=15, alpha=0.5, label='Surowe (Kinect)')

ax.set_title("Ruch Dłoni w Przestrzeni 3D (Wizualizacja Poprawna)")
ax.set_xlabel("Oś X (Lewo/Prawo) [cm]")
ax.set_ylabel("Oś Y (Góra/Dół) [cm]")
ax.set_zlabel("Oś Z (Głębia) [cm]")

ax.legend()


a = []
b = []
c = []

pose = HandPoseFilter()
for value in raw_data_3d:  
    filtered_value = pose.update(value)
    a.append(filtered_value[0] if filtered_value is not None else np.nan)
    b.append(filtered_value[1] if filtered_value is not None else np.nan)
    c.append(filtered_value[2] if filtered_value is not None else np.nan)    


ax.plot(a, b, c, 'g-', linewidth=2, alpha=0.8, label='result')
plt.show()