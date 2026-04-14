import multiprocessing as mp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

def plot_worker(queue, max_points=100):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlabel('Oś X (Lewo/Prawo) [cm]')
    ax.set_ylabel('Oś Z (Głębia) [cm]')
    ax.set_zlabel('Oś Y (Góra/Dół) [cm]')
    ax.set_title('Live Kinect Hand Tracking')
    ax.set_xlim(-50, 50)  
    ax.set_ylim(40, 140) 
    ax.set_zlim(-50, 50)  

    raw_x, raw_y, raw_z = deque(maxlen=max_points), deque(maxlen=max_points), deque(maxlen=max_points)
    filt_x, filt_y, filt_z = deque(maxlen=max_points), deque(maxlen=max_points), deque(maxlen=max_points)
    raw_plot, = ax.plot([], [], [], 'ro', alpha=0.4, markersize=4, label='Raw (Kinect)')
    filt_plot, = ax.plot([], [], [], 'g-', linewidth=2.5, label='Filtered (EMA)')
    ax.legend()

    def update(frame):
        while not queue.empty():
            try:
                data = queue.get_nowait()
                if data == "STOP":
                    plt.close()
                    return
                
                raw, filt = data
                
                if raw is not None:
                    raw_x.append(raw[0] * 100)
                    raw_y.append(raw[2] * 100)
                    raw_z.append(raw[1] * 100)
                
                if filt is not None:
                    filt_x.append(filt[0] * 100)
                    filt_y.append(filt[2] * 100)
                    filt_z.append(filt[1] * 100)
            except:
                pass

        if len(raw_x) > 0:
            raw_plot.set_data(raw_x, raw_y)
            raw_plot.set_3d_properties(raw_z)

        if len(filt_x) > 0:
            filt_plot.set_data(filt_x, filt_y)
            filt_plot.set_3d_properties(filt_z)    
        return raw_plot, filt_plot

    ani = animation.FuncAnimation(fig, update, interval=10, blit=False)
    plt.show()

class DebugWindow:
    def __init__(self, max_points=60):
        self.queue = mp.Queue()
        self.process = mp.Process(target=plot_worker, args=(self.queue, max_points))

    def start(self):
        self.process.start()

    def update(self, raw_point, filtered_point):
        if self.process.is_alive():
            self.queue.put((raw_point, filtered_point))

    def stop(self):
        self.queue.put("STOP")
        self.process.join(timeout=1)