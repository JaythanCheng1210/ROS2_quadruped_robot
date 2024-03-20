from math import sin,cos,pi
import numpy as np
from PuppyPI_Config import PuppyConfiguration, state

class PuppyStance():
    def __init__(self):
        self.config = PuppyConfiguration()
        self.l = self.config.L
        self.b = self.config.B
        self.w = self.config.W

    def cal_attitude(self, r, p, y, x, h):

        pos = np.mat([0.0,  0.0,  h ]).T
        rpy = np.array([-r, p, y]) * pi / 180
        R, P, Y = rpy[0], rpy[1], rpy[2]

        rotx = np.mat([	[	1,		0,			0		],
                        [ 	0,		cos(R),		-sin(R)	],
                        [ 	0,		sin(R),		cos(R)	]])
        roty = np.mat([	[ cos(P),	0,			-sin(P)	],
                        [ 0,		1,			0		],
                        [ sin(P),	0,			cos(P)	]]) 
        rotz = np.mat([	[ cos(Y),	-sin(Y),	0		],
                        [ sin(Y),	cos(Y),		0		],
                        [ 0,		0,			1		]])
        
        rot_mat = rotx * roty * rotz

        body_struc = np.mat([	[	self.l/2,	self.b/2,	0],
                                [	self.l/2,   -self.b/2,	0],
                                [	-self.l/2,	-self.b/2,	0],
                                [	-self.l/2,	self.b/2,	0]]).T
        
        footpoint_struc = np.mat([	[	self.l/2,	self.w/2,	0],
                                    [	self.l/2,	-self.w/2,	0],
                                    [	-self.l/2,	-self.w/2,	0],
                                    [	-self.l/2,	self.w/2,	0]]).T

        AB = np.mat(np.zeros((3, 4)))
        for i in range(4):
            AB[:, i] = - pos - rot_mat * body_struc[:, i] + footpoint_struc[:, i]
        AB = AB.T

        leg_1_x = AB[0,0]-x
        leg_1_y = AB[0,2]

        leg_2_x = AB[1,0]-x
        leg_2_y = AB[1,2]

        leg_3_x = AB[2,0]-x
        leg_3_y = AB[2,2]

        leg_4_x = AB[3,0]-x
        leg_4_y = AB[3,2]
        
        # print("x1:",x1, "y1:", y1)
        # print("x2:",x2, "y2:", y2)
        # print("x3:",x3, "y3:", y3)
        # print("x4:",x4, "y4:", y4)
        
        foot_locations = np.array([ [   leg_1_x,    leg_2_x,    leg_3_x,    leg_4_x],
                                    [  -leg_1_y,   -leg_2_y,   -leg_3_y,   -leg_4_y],
                                    [         0,          0,          0,          0]])
        
        state.foot_locations = foot_locations

        return foot_locations

if __name__ == "__main__":
    pass