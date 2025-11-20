import socket
import threading
import time
from typing import Optional
from pyrobotiqur.enums import GripperStatus, ObjectStatus


class RobotiqGripper:
    """
    Simple Python interface for a Robotiq 2F / Hand-E gripper connected
    to a Universal Robots controller via the Robotiq URCap.

    It talks to the URCap's internal server over a TCP socket (default
    port 63352) using ASCII commands like "GET POS" / "SET POS 100".
    """

    def __init__(self, host, port=63352, timeout=2.0):
        """
        :param host: IP address or hostname of the UR controller
        :param port: TCP port used by the Robotiq URCap server (default 63352)
        :param timeout: socket timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    # --- low level socket helpers -------------------------------------------------

    def connect(self):
        """Open the TCP connection to the UR controller."""
        if self._sock is not None:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect((self.host, self.port))
        self._sock = s

    def disconnect(self):
        """Disconnect the TCP connection."""
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.disconnect()

    def _send_raw(self, cmd):
        """
        Send a raw command string and return the raw response as a decoded string.
        Adds the trailing newline automatically.
        """
        if self._sock is None:
            raise RuntimeError("Socket not connected. Call connect() first.")
        data = (cmd.strip() + "\n").encode("ascii")
        with self._lock:
            self._sock.sendall(data)
            resp = self._sock.recv(1024)
        return resp.decode("ascii").strip()

    # --- variables API (GET / SET) -----------------------------------------------

    def get_var(self, name):
        """
        Read a gripper variable, e.g. POS, SPE, FOR, OBJ, STA, FLT, ...

        :returns: integer value reported by the URCap server
        """
        resp = self._send_raw("GET {}".format(name))
        # Expected format: "<NAME> <value>", e.g. "POS 123"
        parts = resp.split()
        if len(parts) != 2 or parts[0] != name:
            raise ValueError("Unexpected response '{}' when reading {}".format(resp, name))
        return int(parts[1])

    def set_var(self, name, value):
        """
        Write a gripper variable, e.g. POS, SPE, FOR, GTO, ACT, ...

        :returns: None, raises if the server does not reply with 'ack'
        """
        resp = self._send_raw("SET {} {}".format(name, int(value)))
        if resp != "ack":
            raise RuntimeError("SET {} failed, server replied '{}'".format(name, resp))

    # --- convenience wrappers for common variables --------------------------------

    def get_position(self):
        """Current finger position (0 = open, 255 = closed)."""
        return self.get_var("POS")

    def get_requested_position(self):
        """Echo of last commanded position (PRE)."""
        return self.get_var("PRE")

    def get_status(self):
        """High-level gripper status (RESET, ACTIVATING, ACTIVE, ...)."""
        return GripperStatus(self.get_var("STA"))

    def get_object_status(self):
        """Object status (MOVING, contact while opening/closing, at destination)."""
        return ObjectStatus(self.get_var("OBJ"))

    def get_fault(self):
        """Fault code (0 = OK, see Robotiq manual for full list)."""
        return self.get_var("FLT")

    # --- higher-level motion primitives -------------------------------------------

    def reset(self, poll_interval=0.1, timeout=5.0):
        self.set_var("ACT", 0)
        self.set_var("ATR", 0)

        start = time.time()
        while True:
            act = self.get_var("ACT")
            sta = self.get_var("STA")
            if act == 0 and GripperStatus(sta) == GripperStatus.RESET:
                break

            if timeout is not None and (time.time() - start) > timeout:
                raise TimeoutError("Gripper reset did not complete within timeout")

            self.set_var("ACT", 0)
            self.set_var("ATR", 0)
            time.sleep(poll_interval)

    def activate(self, wait=True, poll_interval=0.1):
        """
        Activate the gripper.

        This will:
          * reset the gripper if needed
          * set ACT = 1
          * optionally block until STA == ACTIVE
        """
        # If not already active, go through reset procedure
        if self.get_status() != GripperStatus.ACTIVE:
            self.reset(poll_interval=poll_interval)

        # Request activation
        self.set_var("ACT", 1)

        if wait:
            # Wait until activation completed (ACT=1 and STA=3)
            while not (self.get_var("ACT") == 1 and self.get_status() == GripperStatus.ACTIVE):
                time.sleep(poll_interval)

    def move(self, position, speed=128, force=128,
             wait=True, poll_interval=0.01):
        """
        Move gripper fingers to a position with given speed and force.

        :param position: 0-255 (0 = fully open, 255 = fully closed)
        :param speed:    0-255 (0 = slowest, 255 = fastest)
        :param force:    0-255 (0 = minimum, 255 = maximum)
        :param wait:     if True, block until motion is finished
        :returns: (final_position, ObjectStatus) if wait=True,
                  otherwise (requested_position, ObjectStatus.MOVING)
        """
        # Clamp values to valid range
        position = max(0, min(255, int(position)))
        speed = max(0, min(255, int(speed)))
        force = max(0, min(255, int(force)))

        # Program motion
        self.set_var("POS", position)
        self.set_var("SPE", speed)
        self.set_var("FOR", force)
        self.set_var("GTO", 1)   # go-to start

        if not wait:
            return position, ObjectStatus.MOVING

        # Wait until the gripper has accepted the command (PRE == requested POS)
        while self.get_requested_position() != position:
            time.sleep(poll_interval)

        # Then wait until it stops moving (OBJ != MOVING)
        obj = self.get_object_status()
        while obj == ObjectStatus.MOVING:
            obj = self.get_object_status()
            time.sleep(poll_interval)

        final_pos = self.get_position()
        return final_pos, obj

    def open(self, speed=128, force=1,
             wait=True, poll_interval=0.01):
        """Fully open the gripper."""
        return self.move(0, speed=speed, force=force, wait=wait,
                         poll_interval=poll_interval)

    def close(self, speed=128, force=128,
              wait=True, poll_interval=0.01):
        """Fully close the gripper."""
        return self.move(255, speed=speed, force=force, wait=wait,
                         poll_interval=poll_interval)

    def move_percent(self, percent, speed=128, force=128,
                     wait=True, poll_interval=0.01):
        """
        Move gripper to a position specified in percent.

        :param percent: 0-100  (0% = fully open, 100% = fully closed)
        :param speed:   0-255  (0 = slowest, 255 = fastest)
        :param force:   0-255  (0 = minimum, 255 = maximum)
        :param wait:    if True, block until motion is finished
        :returns: (final_position, ObjectStatus) if wait=True,
                  otherwise (requested_position, ObjectStatus.MOVING)
        """
        # Clamp percentage
        percent = max(0.0, min(100.0, float(percent)))

        # Map 0–100% to 0–255
        position = int(round(percent / 100.0 * 255.0))

        return self.move(position, speed=speed, force=force,
                         wait=wait, poll_interval=poll_interval)
