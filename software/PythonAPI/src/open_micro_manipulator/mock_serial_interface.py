import time
import re
from typing import Optional, Callable, Tuple
from .serial_interface import SerialInterface
from .exceptions import CommandError

class MockSerialInterface:
    def __init__(self, port: str = "mock", baud_rate: int = 115200,
                 command_msg_callback: Optional[Callable] = None,
                 log_msg_callback: Optional[Callable] = None,
                 unsolicited_msg_callback: Optional[Callable] = None,
                 reconnect_timeout: int = 5):
        self.port = port
        self.baud_rate = baud_rate
        self.command_msg_callback = command_msg_callback
        self.log_message_callback = log_msg_callback
        self.unsolicited_msg_callback = unsolicited_msg_callback
        self.is_open = False
        self.position = [0.0, 0.0, 0.0]
        self.connect(reconnect_timeout)

    def connect(self, timeout: float) -> bool:
        # Simulate connection
        self.is_open = True
        return True

    def close(self):
        self.is_open = False

    def send_command(self, cmd: str, timeout: float = 2) -> Tuple[SerialInterface.ReplyStatus, str]:
        if not self.is_open:
             raise CommandError("Serial not open")

        cmd = cmd.strip()
        if self.command_msg_callback:
            self.command_msg_callback(cmd + "\n", None, '')

        # Simulate small delay
        time.sleep(0.01)

        response_status = SerialInterface.ReplyStatus.OK
        response_content = ""
        error_msg = ""

        if cmd.startswith("M58"):
             response_content = "v1.0.1"
        elif cmd.startswith("M50"):
             response_content = f"X{self.position[0]:.6f} Y{self.position[1]:.6f} Z{self.position[2]:.6f}"
        elif cmd.startswith("M51"):
             # Mock M51 response
             lines = []
             for i in range(3):
                 # Fake angle: current position * 10 + offset
                 angle = self.position[i] * 10.0 + (i * 15.0)
                 raw = angle * 100.0
                 lines.append(f"Joint {i}:  {angle:.4f} deg  (raw={raw:.4f})")
             response_content = "\n".join(lines)
        elif cmd.startswith("M53"):
             response_content = "1"
        elif cmd.startswith("G0") or cmd.startswith("G1"):
             # Parse position update
             match_x = re.search(r"X([-+]?\d*\.?\d+)", cmd)
             match_y = re.search(r"Y([-+]?\d*\.?\d+)", cmd)
             match_z = re.search(r"Z([-+]?\d*\.?\d+)", cmd)

             if match_x: self.position[0] = float(match_x.group(1))
             if match_y: self.position[1] = float(match_y.group(1))
             if match_z: self.position[2] = float(match_z.group(1))

        elif cmd.startswith("G28"):
            self.position = [0.0, 0.0, 0.0]

        elif cmd.startswith("M56"):
            # Calibration fake data
            # Format: angle, field_angle, encoder_raw
            # We return 3 columns of data
            # Just generate some dummy data
            lines = []
            for i in range(10):
                angle = i * 0.1
                field = i * 0.1
                raw = i * 100
                lines.append(f"{angle:.4f},{field:.4f},{raw:.4f}")
            response_content = "\n".join(lines)

        elif cmd.startswith("G24"):
             # Set pose
             match_x = re.search(r"X([-+]?\d*\.?\d+)", cmd)
             match_y = re.search(r"Y([-+]?\d*\.?\d+)", cmd)
             match_z = re.search(r"Z([-+]?\d*\.?\d+)", cmd)
             if match_x: self.position[0] = float(match_x.group(1))
             if match_y: self.position[1] = float(match_y.group(1))
             if match_z: self.position[2] = float(match_z.group(1))

        if self.command_msg_callback:
            self.command_msg_callback(response_content, response_status, error_msg)

        return response_status, response_content
