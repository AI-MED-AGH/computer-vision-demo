try:
    import serial
except ImportError:
    serial = None

class ArduinoBridge:
    def __init__(self, port: str, baudrate: int = 115200, enabled: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.enabled = enabled
        self.ser=None

    def connect(self) -> None:
        if not self.enabled:
            return

        if serial is None:
            print("pyserial not installed")
            return

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port}")
        except Exception as e:
            print(f"Failed to connect to {self.port} : {e}")

    def send_joint_angles_deg(self, angles_deg) -> None:
        if not self.enabled or self.ser is None:
            return

        try:
            angles = [int(angle) for angle in angles_deg]
            message = "A," + ",".join(str(angle) for angle in angles) + "\n"
            self.ser.write(message.encode())
        except Exception as e:
            print(f"Failed to send angles to {self.port} : {e}")

    def close(self) -> None:
        if self.ser is None:
            return

        try:
            self.ser.close()
        except Exception as e:
            print(f"Failed to close {self.port} : {e}")
        finally:
            self.ser = None