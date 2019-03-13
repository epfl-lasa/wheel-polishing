#!/usr/bin/env python

'''
Movement of the wheel fro the r5

@author lukashuber
@date 2019-03-01
'''

import rospy
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Quaternion, Pose, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from multiprocessing import Lock

import numpy as np
import numpy.linalg as LA
from numpy import pi

import sys
import signal

import warnings

from class_quaternion import *

from cartesian_state_msgs.msg import PoseTwist

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# MAPING_INPUT = np.array([0,1,2,3,4,5,6])
# MAPING_OUTPUT = np.array([2,1,0,3,4,5,6])

N_JOINTS = 6

class MoveWheel():
    def __init__(self, setUp=""):
        # Initialize variables -- currently constant obstacle geometry
        rospy.init_node("talker_robotWheel", anonymous=True)

        self.mutex = Lock()
        
        self.pub_jointVel = rospy.Publisher("/ur5/ur_driver/joint_speed", JointTrajectory, queue_size=5)
        
        # First subscriber call back variable
        self.recieved_eeVel_msg = False
        self.recieved_jointState_msg = False

        # Initialize topics
        self.sub_eeVel = rospy.Subscriber("/ur5/tool_velocity", TwistStamped, self.callback_eeVel)
        self.sub_joint_pos = rospy.Subscriber("/ur5/joint_states", JointState, self.callback_jointState)

        self.freq = 50
        self.dt = 1./self.freq
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        while not (self.recieved_eeVel_msg and
                   self.recieved_jointState_msg):
            rospy.sleep(0.5)
            print("Waiting for first callbacks...")
            if rospy.is_shutdown():
                print('Shuting down')
                break

    def run(self):
        # Create publisher
        self.it_attr = 0     # only for linear 
        
        self.itCount = 0
        
        if not rospy.is_shutdown():
            print('Start wheel movement.')
            
        # Entering UPDATE loop
        while not rospy.is_shutdown():
            if not (self.active_controlMode in controlModes):
                warnings.warn("Wrong control mode")

            if controlModes[self.active_controlMode] == controlModes["position_polishing"]:
                if not(self.itCount%10):
                    print('update_jointPos')
                self.update_jointPos(self.jointPos_polishing)
                
            self.rate.sleep()  # Wait zzzz*
            
            if not(self.itCount % 100):
                print('Loop #{}'.format(self.itCount))

                # if controlModes[self.active_controlMode] == controlModes["passive_ds"]:
                    # print('vel des:', self.vel_des)
                    # print('angvel des:', ang_vel)
                    # print('norm vel des:', LA.norm(self.vel_des) )
                    # print('pos:', self.position_robo)
                
            self.itCount += 1
        

    def shutdown_command(self, signal, frame):
        print('See you next time')

        msg_jointVel = JointTrajectory()
        msg_jointVel.header.stamp = rospy.Time.now()

        msg_jointVel.joint_names = JOINT_NAMES
        # msg_jointVel.points.velocities = des_joint_vel.tolist()

        newPoint = JointTrajectoryPoint()
        
        msg_jointVel.points.append(newPoint)

        self.pub_jointVel.publish(msg_jointVel)
        
        rospy.signal_shutdown('User shutdown.')
        
        print('shudown finished')

    
    def update_jointPos(self, goal_jointPose, vel_limit=0.2, P=1, I=0, D=0, time_fac=100):
        with self.mutex:
            # PID control
            # TODO - monitor forces
            des_joint_vel = (goal_jointPose - self.joint_pos)*P

            max_des_vel = np.max(np.abs(des_joint_vel)) 
            des_joint_vel = np.min([vel_limit/max_des_vel, max_des_vel])*des_joint_vel

            msg_jointVel = JointTrajectory()
            msg_jointVel.header.stamp = rospy.Time.now()

            msg_jointVel.joint_names = JOINT_NAMES
            # msg_jointVel.points.velocities = des_joint_vel.tolist()

            newPoint = JointTrajectoryPoint()
            if not self.itCount % 10:
                print('joint pos', self.joint_pos)
                print('des_joint_vel', des_joint_vel)
                print('mag_joint_vel', LA.norm(des_joint_vel) )
            
            newPoint.positions = (self.joint_pos + self.dt*des_joint_vel).tolist()
            newPoint.velocities = des_joint_vel.tolist()

            # WARNING: The robot input message and control message have
            # a different mapping this has to be mapped
            # joint mapping [ur5/joint_states -> ur5/ur_driver/joint_speed]
            # ur5_arm_shoulder_pan_joint=2 -> 0
            # ur5_arm_shoulder_lift_joint=1 -> 1
            # ur5_arm_shoulder_lift_shoulder=0 -> 2
            # 3 -> 3
            # 4 -> 4
            # 5 -> 5
            msg_jointVel.joint_names[0], msg_jointVel.joint_names[2] = msg_jointVel.joint_names[2], msg_jointVel.joint_names[0]
            newPoint.positions[0], newPoint.positions[2] = newPoint.positions[2], newPoint.positions[0]
            newPoint.velocities[0], newPoint.velocities[2] = newPoint.velocities[2], newPoint.velocities[0]

            msg_jointVel.points.append(newPoint)

            self.pub_jointVel.publish(msg_jointVel)

    def get_distance_from_attractor(self):
        return LA.norm((self.attractors[:, self.it_attr] - self.position_robo))
    
    
    def callback_eeVel(self, msgs):
        if not self.recieved_eeVel_msg:
            self.recieved_eeVel_msg = True
            print("Recieved first ee velocity")
        
    def callback_jointState(self, msg):
        with self.mutex:
            self.joint_pos = np.array(msg.position[:N_JOINTS])
            self.joint_vel = msg.velocity[:N_JOINTS]
            self.joint_torque = msg.effort[:N_JOINTS]

            if not self.recieved_jointState_msg:
                self.recieved_jointState_msg = True
                print("Recieved first joint state")
                
            
if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)
        
        MoveWheel_inst = MoveWheel('conveyerBelt_basket')
        
        signal.signal(signal.SIGINT, MoveWheel_inst.shutdown_command)
        
        MoveWheel_inst.run()
        
    except rospy.ROSInterruptException:
        pass
