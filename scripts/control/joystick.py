#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
rostopic
Backward:
    rostopic pub /drrobot_jaguar_v6_basemotor_cmd drrobot_jaguar_v6/BaseMotorCmd -- 100 -100
Forward:
    rostopic pub /drrobot_jaguar_v6_basemotor_cmd drrobot_jaguar_v6/BaseMotorCmd -- -100 100
    
Msg info:

#this is used for Base(wheel/Track) motor command, the motor is driven in open-loop mode
int32 leftCmd	  # left track/wheel drive command, it should be PWM value(-1000 ~ 1000)
int32 rightCmd	  # right track/wheel drive command, it should be PWM value(-1000 ~ 1000)

#this is used for Flip Motor Drive Command, all the motors work in position-control mode
int32 leftFrontCmd	# left front flip motor cmd, will send to drive baord-3, channel 1, like "!PR 1 -30"
int32 rightFrontCmd	# right front flip motor cmd, will send to drive baord-3, channel 2, like "!PR 2 50"
int32 leftRearCmd	# left rear flip motor cmd, will send to drive baord-4, channel 1, like "!PR 1 0", will keep same position
int32 rightRearCmd	# right rear flip motor cmd, will send to drive baord-4, channel 2, like "!PR 2 -100"

'''

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Joy
import time
from std_srvs.srv import Empty, EmptyRequest
from tracked_irl.msg import BaseMotorCmd, FlipMotorCmd

class JoystickControl:
    def __init__(self):
        self.init_time = 0
        self.last_vel = 0
        rospy.init_node('joystick_control', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=1)
        self.pub_track_cmd = rospy.Publisher('/drrobot_jaguar_v6_basemotor_cmd', BaseMotorCmd, queue_size=1)
        self.pub_flip_cmd = rospy.Publisher('/drrobot_jaguar_v6_flipmotor_cmd', FlipMotorCmd, queue_size=1)
        self.pwm = 0.1
        self.msg_tracks = BaseMotorCmd()
        self.msg_flips = FlipMotorCmd()
        self.state_flips = {
            'leftFront': 0,
            'rightFront': 0,
            'leftRear': 0,
            'rightRear': 0
        }
        rospy.spin()

    def sign(self, var):
        if var == 0:
            return 1
        return var/abs(var)

    def joystick_callback(self, msg):
        
        # Initialization
        PWM_MAX = 1000
        STEP_FLIP = 125
        # Flippers        
        print('buttons \n {}'.format(msg.buttons))
        self.state_flips['leftRear'] +=  STEP_FLIP * (-msg.buttons[0] + msg.buttons[1])
        self.state_flips['leftFront'] +=  STEP_FLIP * (-msg.buttons[2] + msg.buttons[3])
        # self.state_flips['rightRear'] +=  sign * STEP_FLIP * msg.buttons[0] 
        # self.state_flips['rightFront'] +=  sign * STEP_FLIP * msg.buttons[0]  
        self.msg_flips.leftRearCmd = self.state_flips['leftRear']
        self.msg_flips.rightRearCmd = self.state_flips['leftRear']
        self.msg_flips.leftFrontCmd = self.state_flips['leftFront']
        self.msg_flips.rightFrontCmd = self.state_flips['leftFront']

        # Tracks
        axe0 = msg.axes[0]
        axe1 = msg.axes[1]

        self.pwm += msg.axes[7]*0.1 if msg.axes[7]*0.1 < 0.5 else 0.5
        print(self.pwm)
        self.msg_tracks.leftCmd = -int(PWM_MAX * self.sign(axe1 - axe0) * min(abs(axe0 - axe1), 1)) * self.pwm
        self.msg_tracks.rightCmd = int(PWM_MAX * self.sign(axe1 + axe0) * min(abs(axe0 + axe1), 1)) * self.pwm
        self.pub_flip_cmd.publish(self.msg_flips)
        self.pub_track_cmd.publish(self.msg_tracks)



if __name__ == '__main__':
    JoystickControl()
