import serial

class ArduinoBridge:
    def __init__(self, port: str, baudrate: int = 115200, enabled: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.enabled = enabled
        self.ser=None

    def connect(self) -> None:
        if not self.enabled:
            return
        else:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                print(f"Connected to {self.port}")
            except Exception as e:
                print(f"Failed to connect to {self.port} : {e}")

    def send_joint_angles_deg(self, angles_deg) -> None:

        pass

    def close(self) -> None:

        pass