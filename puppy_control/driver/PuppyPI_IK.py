from math import asin,acos,atan,pi,sqrt
from PuppyPI_Config import PuppyConfiguration, PuppyState, state
import numpy as np

# Servo: 
# 
#  (o o)
# 1-----2
#   |||
#   |||
# 4-----3
#    *
#    *

class PuppyIK():
    def __init__(self):
        self.config = PuppyConfiguration()
        self.upper_leg_length = self.config.upper_leg_length
        self.lower_leg_length = self.config.lower_leg_length

    def leg_inverse_kinematics(self, foot_locations):

        leg_1_x, leg_2_x, leg_3_x, leg_4_x = foot_locations[0, 0], foot_locations[0, 1], foot_locations[0, 2], foot_locations[0, 3]
        leg_1_y, leg_2_y, leg_3_y, leg_4_y = foot_locations[1, 0], foot_locations[1, 1], foot_locations[1, 2], foot_locations[1, 3]

        #Leg1
        leg_1_x = -leg_1_x
        lower_leg_1 = pi-acos((leg_1_x**2+leg_1_y**2-self.upper_leg_length**2-self.lower_leg_length**2)/(-2*self.upper_leg_length**2))
        phi_1 = acos((self.upper_leg_length**2+leg_1_x**2+leg_1_y**2-self.lower_leg_length**2)/(2*self.upper_leg_length*sqrt(leg_1_x**2+leg_1_y**2)))
        
        if leg_1_x>0:
            upper_leg_1 = (abs(atan(leg_1_y/leg_1_x))-phi_1)
        elif leg_1_x < 0:
            upper_leg_1 = (pi-abs(atan(leg_1_y/leg_1_x))-phi_1)
        else:
            upper_leg_1 = (pi-1.5707-phi_1)

        upper_leg_1= (pi - upper_leg_1)
        lower_leg_1 = lower_leg_1 - pi
        
        #Leg2
        leg_2_x = -leg_2_x
        lower_leg_2 = pi-acos((leg_2_x**2+leg_2_y**2-self.upper_leg_length**2-self.lower_leg_length**2)/(-2*self.upper_leg_length**2))
        phi_2 = acos((self.upper_leg_length**2+leg_2_x**2+leg_2_y**2-self.lower_leg_length**2)/(2*self.upper_leg_length*sqrt(leg_2_x**2+leg_2_y**2)))
        
        if leg_2_x>0:
            upper_leg_2=abs(atan(leg_2_y/leg_2_x))-phi_2
        elif leg_2_x<0:
            upper_leg_2=pi-abs(atan(leg_2_y/leg_2_x))-phi_2
        else:
            upper_leg_2=pi-1.5707-phi_2

        upper_leg_2 = -(upper_leg_2 -pi)
        lower_leg_2 = -(pi - lower_leg_2)

        #Leg3
        leg_3_x = -leg_3_x
        lower_leg_3 = pi-acos((leg_3_x**2+leg_3_y**2-self.upper_leg_length**2-self.lower_leg_length**2)/(-2*self.upper_leg_length**2))
        phi_3 = acos((self.upper_leg_length**2+leg_3_x**2+leg_3_y**2-self.lower_leg_length**2)/(2*self.upper_leg_length*sqrt(leg_3_x**2+leg_3_y**2)))
        
        if leg_3_x > 0:
            upper_leg_3 = abs(atan(leg_3_y/leg_3_x))-phi_3
        elif leg_3_x < 0:
            upper_leg_3 = pi-abs(atan(leg_3_y/leg_3_x))-phi_3
        else:
            upper_leg_3 = pi-1.5707-phi_3

        upper_leg_3 = - (upper_leg_3 -pi)
        lower_leg_3 = -(pi - lower_leg_3)

        #Leg4
        leg_4_x = -leg_4_x
        lower_leg_4 = pi-acos((leg_4_x**2+leg_4_y**2-self.upper_leg_length**2-self.lower_leg_length**2)/(-2*self.upper_leg_length**2))
        phi_4 = acos((self.upper_leg_length**2+leg_4_x**2+leg_4_y**2-self.lower_leg_length**2)/(2*self.upper_leg_length*sqrt(leg_4_x**2+leg_4_y**2)))
        
        if leg_4_x > 0:
            upper_leg_4 = abs(atan(leg_4_y/leg_4_x))-phi_4
        elif leg_4_x < 0:
            upper_leg_4 = pi-abs(atan(leg_4_y/leg_4_x))-phi_4
        else:
            upper_leg_4 = pi-1.5707-phi_4

        
        upper_leg_4 = pi-upper_leg_4
        lower_leg_4 = lower_leg_4 -pi

        joint_angles = np.array([   [  0.,  0.,  0.,  0.],
                                    [upper_leg_1, upper_leg_2, upper_leg_3, upper_leg_4],
                                    [-lower_leg_1, -lower_leg_2, -lower_leg_3, -lower_leg_4]])

        state.joint_angles = joint_angles
        return joint_angles
    

if __name__ == "__main__":
    # test code 
    from PuppyPI_ServoPWM import ServoPWM
    import time
    puppy_IK = PuppyIK()

    # leg_position = [30, 30, 30, 30, -100, -100, -100, -100]
    foot_locations = np.array([[ 30,   30,    30,    30],
                            [-100,  -100,   -100,   -100],
                            [       0,         0,          0,          0]])
    joint_angles = puppy_IK.leg_inverse_kinematics(foot_locations)

    # servo_PWM = ServoPWM()

    # angle = 100 * pi /180
    # servo_PWM.send_servo_command(joint_angles[1, 3], 1, 3)
    # servo_PWM.send_servo_command(joint_angles[2, 3], 2, 3)
    # servo_PWM.set_actuator_positions(joint_angles)
    time.sleep(0.2)

    print(joint_angles[1, 0], joint_angles[2, 0])
    print(joint_angles[1, 1], joint_angles[2, 1])
    print(joint_angles[1, 2], joint_angles[2, 2])
    print(joint_angles[1, 3], joint_angles[2, 3])