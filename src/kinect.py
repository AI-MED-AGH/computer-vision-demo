import time
import math
import numpy as np

class KinectHandTracker:
    def __init__(self, use_mock: bool =True):
        self.use_mock = use_mock
        self.sim_t = 0.0
        self.device_handle = None  # Miejsce na uchwyt do rzeczywistego urządzenia / SDK
        
        self._is_running = False
        self._last_time = time.time()

    def start(self) -> None:
        self._is_running = True
        self._last_time = time.time()
        
        if not self.use_mock:
            # TODO: Inicjalizacja SDK Kinecta (np. podłączenie do kamery, start strumieni)
            pass
            
        print(f"[KinectHandTracker] Uruchomiono (Mock: {self.use_mock})")

    def stop(self) -> None:

        self._is_running = False
        
        if not self.use_mock:
            # TODO: Zamknięcie strumieni i zwolnienie zasobów urządzenia
            self.device_handle = None
            
        print("[KinectHandTracker] Zatrzymano")

    def get_hand_position_3d(self):
        if not self._is_running:
            return None

        if self.use_mock:
            # Obliczanie upływu czasu (delta time) do płynnej animacji
            current_time = time.time()
            dt = current_time - self._last_time
            self._last_time = current_time
            self.sim_t += dt

            # Generowanie płynnego ruchu w 3D (figura podobna do krzywej Lissajous)
            x = 0.3 * math.sin(self.sim_t * 1.2)        # Ruch lewo-prawo
            y = 0.2 * math.cos(self.sim_t * 0.8)        # Ruch góra-dół
            z = 1.0 + 0.15 * math.sin(self.sim_t * 0.5) # Zmiana głębi (odległość ok. 1m)

            return np.array([x, y, z], dtype=float)

        else:
            # Tryb rzeczywisty z zabezpieczeniem przed crashem
            try:
                color_frame = self._get_color_frame()
                depth_frame = self._get_depth_frame()

                if color_frame is None or depth_frame is None:
                    return None

                hand_2d = self._detect_hand_2d(color_frame)
                if hand_2d is None:
                    return None

                u, v = hand_2d
                
                # Pobranie wartości głębi (zakładamy, że depth_frame to tablica numpy 2D)
                # Należy pamiętać, że współrzędne obrazu to (v: wiersz, u: kolumna)
                depth_value = depth_frame[v, u] 

                # Brak poprawnych danych głębi w danym pikselu (częsty przypadek w sensorach)
                if depth_value <= 0:
                    return None

                point_3d = self._project_to_3d(u, v, depth_value)
                return point_3d

            except Exception as e:
                # Moduł nie crashuje przy pojedynczym błędzie klatki
                print(f"[KinectHandTracker] Błąd przetwarzania klatki: {e}")
                return None

    def _get_color_frame(self):
        pass

    def _get_depth_frame(self):
        pass

    def _detect_hand_2d(self, color_frame):

        pass

    def _project_to_3d(self, u, v, depth_value):

        pass