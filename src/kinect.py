import json
import socket
from typing import Optional

import numpy as np


class KinectHandTracker:
    def __init__(
        self,
        udp_ip: str = "0.0.0.0",
        udp_port: int = 5005,
        timeout_s: float = 0.2,
        hand_key: str = "hand_right",
    ):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.timeout_s = timeout_s
        self.hand_key = hand_key

        self.sock = None
        self._is_running = False

        self.last_position: Optional[np.ndarray] = None
        self.last_hand_state: Optional[int] = None
        self.last_hand_confidence: Optional[int] = None

    def start(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(self.timeout_s)
        self._is_running = True
        print(f"Kinect receiver listening on {self.udp_ip}:{self.udp_port}")

    def stop(self) -> None:
        self._is_running = False
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def get_hand_measurement(self) -> Optional[dict]:
        if not self._is_running or self.sock is None:
            return None

        try:
            packet, _ = self.sock.recvfrom(4096)
            decoded = packet.decode("utf-8").strip()
            data = json.loads(decoded)

            tracked = data.get("tracked", True)
            if not tracked:
                return None

            hand = data.get(self.hand_key)
            if hand is None:
                return None

            point = np.asarray(hand, dtype=float).reshape(3)
            if not np.isfinite(point).all():
                return None

            hand_state = data.get("hand_state", None)
            hand_confidence = data.get("hand_confidence", None)

            self.last_position = point
            self.last_hand_state = hand_state
            self.last_hand_confidence = hand_confidence

            return {
                "position": point,
                "hand_state": hand_state,
                "hand_confidence": hand_confidence,
            }

        except socket.timeout:
            return None

        except json.JSONDecodeError as e:
            print("receiver json error:", e)
            try:
                print("bad packet:", packet.decode("utf-8", errors="ignore"))
            except Exception:
                pass
            return None

        except Exception as e:
            print("receiver error:", e)
            return None

    def get_hand_position_3d(self) -> Optional[np.ndarray]:
        measurement = self.get_hand_measurement()
        if measurement is None:
            return None
        return measurement["position"]