import time

from PuppyPI_IK import PuppyIK
from PuppyPI_GAIT import PuppyGait
from PuppyPI_Stance import PuppyStance
from PuppyPI_ServoPWM import ServoPWM
from PuppyPI_Config import PuppyState, PuppyConfiguration, state
from PuppyPI_ActionGroups import action_dic
import numpy as np

class ControlCmd:
    def __init__(self):
        self.servo_PWM = ServoPWM()
        self.IK = PuppyIK()
        self.gait = PuppyGait()
        self.stance = PuppyStance()
        self.config = PuppyConfiguration()
        self.h = self.config.puppy_height
        self.t = 0
        self.dt = 0.01
        self.speed = 1.5
        self.joint_angles_path = []
        
        self.init_stance()

    def puppy_move(self, left_leg_move, right_leg_move, leg_height = 15):
        while self.t <= 1:
            foot_locations = self.gait.trot(self.t, self.speed*25, leg_height, left_leg_move, left_leg_move, right_leg_move, right_leg_move)

            if   (left_leg_move+right_leg_move) > 0:
                foot_locations_offset = self.stance.cal_attitude(0, 0, 0, self.speed*10, self.h)
                foot_locations[0] += foot_locations_offset[0]

            elif (left_leg_move+right_leg_move) < 0:
                foot_locations_offset = self.stance.cal_attitude(0, 0, 0, self.speed*20, self.h)
                foot_locations[0] -= foot_locations_offset[0]

            elif (left_leg_move+right_leg_move) == 0:    #原地旋转
                foot_locations_offset = self.stance.cal_attitude(0, 0, 0, self.speed*10, self.h)
                foot_locations[0] -= foot_locations_offset[0]

            else:
                pass

            joint_angles = self.IK.leg_inverse_kinematics(foot_locations)
            self.servo_PWM.set_actuator_positions(joint_angles)
            time.sleep(0.0005)
            self.t += self.dt
        self.t = 0

    def stance_control(self, roll, pitch):
        foot_locations = self.stance.cal_attitude(roll, pitch, 0, 0, -80)
        joint_angles = self.IK.leg_inverse_kinematics(foot_locations)
        self.servo_PWM.set_actuator_positions(joint_angles)

    def pose_control(self, target_foot_locations, target_use_time = 1000):
        timestamp = 10
        use_time = 0
        displacement = (target_foot_locations-state.foot_locations)/(target_use_time/timestamp)

        while use_time < target_use_time:
            next_foot_locations = state.foot_locations + displacement
            state.foot_locations = next_foot_locations
            joint_angles = self.IK.leg_inverse_kinematics(next_foot_locations)
            self.servo_PWM.set_actuator_positions(joint_angles)
            time.sleep(0.01)
            use_time += timestamp

    def run_action(self, action_list, action_num ):
            foot_locations = np.zeros(12)
            for i in range(0, 10):
                value = action_list[action_num][i+2]
                foot_locations[i] = float(value)
            foot_locations = foot_locations.reshape(3,4)
            self.pose_control(foot_locations, target_use_time = action_list[action_num][1])

    def init_stance(self):
        for i in range(40,85,5):
            leg_pos = self.stance.cal_attitude(0, 0, 0, i-65, -i)
            joint_angles = self.IK.leg_inverse_kinematics(leg_pos)
            self.servo_PWM.set_actuator_positions(joint_angles)
            time.sleep(0.05)

if __name__ == "__main__":
    pass
    control_cmd = ControlCmd()
    # time.sleep(1)
    # # target = np.array([[   0.,    -40.,    -40.,    0.],
    # #                                     [-50., -40., -40., -50.],
    # #                                     [   0.,    0.,    0.,    0.]])
    # # control_cmd.pose_control(target)

    # # act = np.array([1, 300, -2.0, 0.0, -14.0, -2.0, 0.0, -14.0, 6.0, 0.0, -10.0, 6.0, 0.0, -10.0])
    # act = [1, 300, -20.0, -20.0, 60.0, 60.0, -140.0, -140.0, -100.0, -100.0, 0.0, 0.0, 0.0, 0.0]
    # rotated_foot_locations = np.zeros(12)
    # print(rotated_foot_locations)
    # for i in range(0, len(act)-2):
    #     value = act[i+2]
    #     print("value:", value)
    #     rotated_foot_locations[i] = float(value)
    #     print(i)
    # print("finished")
    # rotated_foot_locations = rotated_foot_locations.reshape(3,4)
    # # rotated_foot_locations = rotated_foot_locations.T
    # print(rotated_foot_locations)

    # print(act[2])
    # foot_locations = np.array([ [ act[2],   act[5],    act[8],    act[11]],
    #                         [act[4],  act[7],   act[10],   act[13]],
    #                         [       0,         0,          0,          0]])*10
    # # control_cmd.pose_control(foot_locations, act[1])
    # time.sleep(0)

    # act = np.array([2, 300, -2.0, 0.0, -14.0, -2.0, 0.0, -14.0, 3.0, 0.0, -14.0, 3.0, 0.0, -14.0])
    # foot_locations = np.array([ [ act[2],   act[5],    act[8],    act[11]],
    #                         [act[4],  act[7],   act[10],   act[13]],
    #                         [       0,         0,          0,          0]])*10
    # control_cmd.pose_control(foot_locations, act[1])
    # time.sleep(0)

    # act = np.array([3, 300, 2.0, 0.0, -8.0, 2.0, 0.0, -8.0, -4.0, 0.0, -12.0, -4.0, 0.0, -12.0])
    # foot_locations = np.array([ [ act[2],   act[5],    act[8],    act[11]],
    #                         [act[4],  act[7],   act[10],   act[13]],
    #                         [       0,         0,          0,          0]])*10
    # control_cmd.pose_control(foot_locations, act[1])

    # time.sleep(2)
    # act = np.array([1, 600, -2.0, 0.0, -14.0, -2.0, 0.0, -14.0, 6.0, 0.0, -10.0, 6.0, 0.0, -10.0])
    # foot_locations = np.array([ [ act[2],   act[5],    act[8],    act[11]],
    #                         [act[4],  act[7],   act[10],   act[13]],
    #                         [       0,         0,          0,          0]])*10
    # control_cmd.pose_control(foot_locations, act[1])
    # time.sleep(0)

    # act = np.array([1, 500, 0.0, 0.0, -8.0, 0.0, 0.0, -8.0, 0.0, 0.0, -8.0, 0.0, 0.0, -8.0])
    # foot_locations = np.array([ [ act[2],   act[5],    act[8],    act[11]],
    #                         [act[4],  act[7],   act[10],   act[13]],
    #                         [       0,         0,          0,          0]])*10
    # control_cmd.pose_control(foot_locations, act[1])
    # time.sleep(2)

    # control_cmd.stance_control(0, 0)
    # control_cmd.stance_control(0, 0)
    # control_cmd.stance_control(0, 0)
    # control_cmd.stance_control(0, 0)
    # for i in range(0, 20, 2):
    #     control_cmd.stance_control(i, 0)
    #     time.sleep(0.1)
    # for i in range(0, 20, 2):
    #     control_cmd.stance_control(0, i)
    #     time.sleep(0.1)
    # for i in range(0, 20):
    #     control_cmd.stance_control(-i, 0)
    #     time.sleep(0.1)
    # for i in range(0, 20):
    #     control_cmd.stance_control(0, -i)
    #     time.sleep(0.1)
    # control_cmd.stance_control(0, 1)
    # control_cmd.stance_control(-5, 0)
    # control_cmd.stance_control(0, -20)
    # control_cmd.stance_control(0, 0)
    # control_cmd.stance_control(0, 0)
    # controlcmd.allServoRelease()
    # t = 0
    # dt = 0.01
    while True:
            control_cmd.puppy_move(1, 1, 12)
            time.sleep(0.001)
            # print(state.joint_angles)
            # print(state.foot_locations)