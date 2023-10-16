#!/usr/bin/env python3
import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time

from keyboard_listener import KeyboardListener

#######################################################
"""This can control and query the LEAP Hand

I recommend you only query when necessary and below 90 samples a second.  Each of position, velociy and current costs one sample, so you can sample all three at 30 hz or one at 90hz.

#Allegro hand conventions:
#0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more
#http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Joint_Zeros_and_Directions_Setup_Guide I belive the black and white figure (not blue motors) is the zero position, and the + is the correct way around.  LEAP Hand in my videos start at zero position and that looks like that figure.

#LEAP hand conventions:
#180 is flat out for the index, middle, ring, fingers, and positive is closing more and more.

"""


########################################################
class LeapNode:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))

        # You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        try:
            self.dxl_client = DynamixelClient(motors, "/dev/ttyUSB0", 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, "/dev/ttyUSB1", 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, "COM13", 4000000)
                self.dxl_client.connect()
        # Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(
            motors, np.ones(len(motors)) * self.kP, 84, 2
        )  # Pgain stiffness
        self.dxl_client.sync_write(
            [0, 4, 8], np.ones(3) * (self.kP * 0.75), 84, 2
        )  # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(
            motors, np.ones(len(motors)) * self.kI, 82, 2
        )  # Igain
        self.dxl_client.sync_write(
            motors, np.ones(len(motors)) * self.kD, 80, 2
        )  # Dgain damping
        self.dxl_client.sync_write(
            [0, 4, 8], np.ones(3) * (self.kD * 0.75), 80, 2
        )  # Dgain damping for side to side should be a bit less
        # Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    # read position
    def read_pos(self):
        return self.dxl_client.read_pos()

    # read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()

    # read current
    def read_cur(self):
        return self.dxl_client.read_cur()


# init the node
def main(**kwargs):
    listener = KeyboardListener()
    leap_hand = LeapNode()

    curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16), zeros=False)
    curr_pos = np.array(curr_pos)
    pos_stride = 0.1

curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16), zeros=False)
curr_pos = np.array(curr_pos)
pos_stride = 0.1

add_key_mappings = {
    "q": 0,
    "a": 1,
    "z": 2,
    "e": 3,
    "d": 4,
    "c": 5,
    "t": 6,
    "g": 7,
    "b": 8,
    "u": 9,
    "j": 10,
    "m": 11,
    "o": 12,
    "l": 13,
    "[": 14,
    "-": 15,
}

minus_key_mappings = {
    "w": 0,
    "s": 1,
    "x": 2,
    "r": 3,
    "f": 4,
    "v": 5,
    "y": 6,
    "h": 7,
    "n": 8,
    "i": 9,
    "k": 10,
    ",": 11,
    "p": 12,
    ";": 13,
    "]": 14,
    "=": 15,
}

frequency = 20
while True:
    try:
        if listener.key in add_key_mappings:
            index = add_key_mappings[listener.key]
            curr_pos[index] += pos_stride
        elif listener.key in minus_key_mappings:
            index = minus_key_mappings[listener.key]
            curr_pos[index] -= pos_stride
        else:
            continue
            
        leap_hand.set_leap(curr_pos)
        print("Position: " + str(leap_hand.read_pos()))
        
        listener.key = ""
        time.sleep(1 / frequency)
    except KeyboardInterruptError:
        break


if __name__ == "__main__":
    main()
