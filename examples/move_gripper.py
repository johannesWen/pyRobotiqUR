import time
from pyrobotiqur import RobotiqGripper


UR_IP = "192.168.0.10"   # <-- replace with your UR controller IP


if __name__ == "__main__":
    with RobotiqGripper(UR_IP) as g:
        # Activate gripper (if not already active)
        g.activate()

        print("Opening gripper...")
        g.open(speed=128, force=10)
        time.sleep(3.0)
        print("POS after open:", g.get_position())

        print("Closing gripper...")
        g.close(speed=200, force=200)
        time.sleep(3.0)
        print("POS after close:", g.get_position())

        print("Moving gripper to position 50%...")
        g.move_percent(50, speed=128, force=128)
        time.sleep(3.0)
        print("POS after move_percent:", g.get_position())

        print("Object status:", g.get_object_status())
        print("Fault code:", g.get_fault())
