import threading
import time
from enum import Enum

import logging
from typing import Optional, Callable, Tuple
import serial
from .exceptions import DeviceNotFoundError, CommandTimeoutError, CommandError

logger = logging.getLogger(__name__)

class SerialInterface:

    class ReplyStatus(Enum):
        OK = 'ok'
        ERROR = 'error'
        TIMEOUT = 'timeout'
        BUSY = 'busy'

    class LogLevel(Enum):
        DEBUG = 'debug'
        INFO = 'info'
        WARNING = 'warning'
        ERROR = 'error'

    # Static mapping from prefix to LogLevel
    log_level_prefix_map = {
        "D)": LogLevel.DEBUG,
        "I)": LogLevel.INFO,
        "W)": LogLevel.WARNING,
        "E)": LogLevel.ERROR,
    }

    def __init__(self, port: str, baud_rate: int = 115200,
                 command_msg_callback: Optional[Callable] = None,
                 log_msg_callback: Optional[Callable] = None,
                 unsolicited_msg_callback: Optional[Callable] = None,
                 reconnect_timeout: int = 5):
        """
        Initializes the serial connection and starts background reader.
        :param port: Serial port name (e.g., 'COM3' or '/dev/ttyUSB0').
        :param baud_rate: Serial baud rate.
        :param log_msg_callback: called when a log message is received
        :param unsolicited_msg_callback: Optional function to call with unsolicited messages.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.reconnect_timeout = reconnect_timeout
        self.serial = None  # initialized on connect

        self.command_msg_callback = command_msg_callback
        self.log_message_callback = log_msg_callback
        self.unsolicited_msg_callback = unsolicited_msg_callback

        # Synchronization for blocking send/receive
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._waiting_for_response = False
        self._response_string = ""
        self._response_status = None
        self._response_error_msg = None

        self.connect(self.reconnect_timeout)

        # Start reader thread
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()


    def connect(self, timeout: float) -> bool:
        """
        Try to open the serial port. Retry until timeout expires.
        """
        deadline = time.time() + timeout
        logger.info(f"Connecting to port '{self.port}'...")
        while time.time() < deadline:
            try:
                self.serial = serial.Serial(self.port, self.baud_rate, timeout=2)
                logger.info(f"Connected to '{self.port}'")
                return True
            except (serial.SerialException, OSError) as e:
                time.sleep(0.2)

        self.serial = None
        raise DeviceNotFoundError(f"Could not connect to port {self.port}")

    def _reader_loop(self):
        """
        Asynchronous reader loop, collecting serial data into a buffer
        """
        buffer = ""
        while True:
            try:
                if self.serial is not None and self.serial.in_waiting:
                    char = self.serial.read(1).decode('ascii', errors='ignore')
                    if char in ['\n', '\r']:
                        if len(buffer) > 0:
                            self._handle_line(buffer)
                            buffer = ""
                    else:
                        buffer += char
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError) as e:
                logger.error(f"Lost connection: {e}")
                try:
                    if self.serial is not None and self.serial.is_open:
                        self.serial.close()
                except Exception:
                    pass

                self.serial = None
                self.connect(self.reconnect_timeout)

    def _handle_line(self, line: str):
        """
        Handles a single serial line sent by the device
        :param line: string containing a single line
        """
        with self._lock:
            log_level, log_msg = self._check_log_msg(line)
            # print(line)
            # log message
            if log_level is not None:
                if self.log_message_callback: self.log_message_callback(log_level, log_msg)
            # response
            elif self._waiting_for_response:
                line_lower = line.lower()
                if line_lower.startswith("ok"):
                    self._response_status = SerialInterface.ReplyStatus.OK
                elif line_lower.startswith("busy"):
                    self._response_status = SerialInterface.ReplyStatus.BUSY
                elif line_lower.startswith("error"):
                    self._response_status = SerialInterface.ReplyStatus.ERROR
                    parts = line.split(":", 1)
                    self._response_error_msg = parts[1].strip() if len(parts) > 1 else ""

                if self._response_status is not None:
                    self._condition.notify()
                else:
                    self._response_string += line + '\n'

            # unsolicited message
            else:
                if self.unsolicited_msg_callback: self.unsolicited_msg_callback(line)

    def _check_log_msg(self, msg: str):
        if len(msg) < 2:
            return None, ''
        return self.log_level_prefix_map.get(msg[:2]), msg[2:]

    def send_command(self, cmd: str, timeout: float = 2) -> Tuple[ReplyStatus, str]:
        """
        Sends a command and blocks until 'ok' or 'error' is received.
        :param cmd: The command to send.
        :param timeout: Maximum time to wait for response.
        :return: Tuple containing Status enum (OK | ERROR | TIMEOUT), and response lines.
        """
        with self._lock:
            if not self.serial or not self.serial.is_open:
                raise CommandError('Serial not open')

            # Reset state
            self._waiting_for_response = True
            self._response_string = ""
            self._response_error_msg = ""
            self._response_status = None

            cmd = (cmd.strip() + "\n")
            self.command_msg_callback(cmd, None, '')

            # Send command
            self.serial.write(cmd.encode('ascii'))
            self.serial.flush()

            # Wait for completion
            end_time = time.time() + timeout
            while self._response_status is None:
                remaining = end_time - time.time()
                if remaining <= 0:
                    self._waiting_for_response = False
                    logger.warning("Command timeout, device didn't reply in time")
                    raise CommandTimeoutError(f"Command timeout, device didn't reply in time")
                self._condition.wait(timeout=remaining)

            self._waiting_for_response = False
            self.command_msg_callback(self._response_string, self._response_status, self._response_error_msg)

            if self._response_status == SerialInterface.ReplyStatus.ERROR:
                raise CommandError(self._response_error_msg)

            return self._response_status, self._response_string

    def close(self):
        """Closes the serial port."""
        if self.serial and self.serial.is_open:
            self.serial.close()
