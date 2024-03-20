

import numpy as np
from enum import Enum

MICROS_PER_RAD = 11.333 * 180.0 / np.pi
NEUTRAL_ANGLE_DEGREES = np.array(  [[  0.,  0.,  0.,  0.],
                                    [ 90., 90., 90., 90.],
                                    [ 90., 90., 90., 90.]])
                                    
class PWMParams:
    def __init__(self):

        PWMServo_IO_dict = {1:4, 2:18, 3:27, 4:10, 5:20, 6:19, 7:13, 8:6, 9:11, 10:5}
        self.pins = np.array(  [[PWMServo_IO_dict[10],  PWMServo_IO_dict[10],   PWMServo_IO_dict[10],   PWMServo_IO_dict[10]],    # abduction servo
                                [PWMServo_IO_dict[3],   PWMServo_IO_dict[1],    PWMServo_IO_dict[5],    PWMServo_IO_dict[7]],    # higher leg servo
                                [PWMServo_IO_dict[4],   PWMServo_IO_dict[2],    PWMServo_IO_dict[6],    PWMServo_IO_dict[8]]])   # lower leg servo
        self.range = 4000
        self.freq = 250

class ServoParams:
    def __init__(self):
        self.neutral_position_pwm = 1500  # Middle position
        self.micros_per_rad = MICROS_PER_RAD  # Must be calibrated

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = NEUTRAL_ANGLE_DEGREES

        self.servo_multipliers = np.array([[ 1,  1,  1,  1], 
                                           [ 1, -1, -1,  1], 
                                           [-1,  1,  1, -1]])

    @property
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians

class PuppyState:
    def __init__(self):
        self.yaw_rate = 0.0
        self.target_height = 16
        self.pitch = 0.0
        self.roll = 0.0

        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))

# TODO: Revise this class
class PuppyConfiguration:
    def __init__(self):
        self.upper_leg_length = 76
        self.lower_leg_length = 83
        self.puppy_height = 100

        self.L = 157
        self.H = self.puppy_height
        self.B = 115
        self.W = 115

        self.dt = 0.01

state = PuppyState()
config = PuppyConfiguration()