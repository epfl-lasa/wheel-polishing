#!/usr/bin/env python

'''
Movement of the wheel fro the r5

@author lukashuber
@date 2019-03-01
'''


import rospy
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Quaternion, Pose, PoseStamped
# import geometry_msgs.msg as geometry_msg
from nav_msgs.msg import Path
# from std_msgs.msg import Float64MultiArray
# from std_msgs.msg import std_msg
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

# JOINT_NAMES = ['ur5_arm_elbow_joint', 'ur5_arm_shoulder_lift_joint', 'ur5_arm_shoulder_pan_joint', 'ur5_arm_wrist_1_joint', 'ur5_arm_wrist_2_joint', 'ur5_arm_wrist_3_joint']
# joint mapping [ur5/joint_states -> ur5/ur_driver/joint_speed]
# ur5_arm_shoulder_pan_joint=2 -> 0
# ur5_arm_shoulder_lift_joint=1 -> 1
# ur5_arm_shoulder_lift_shoulder=0 -> 2
# 3 -> 3
# 4 -> 4
# 5 -> 5



# TODO - replace with other data structure
controlModes = {"joint_pos" : 0,
                "gravity_comp" : 1,
                "motion_upDown" : 2,
                "motion_circular" : 3,
                "motion_leftRight" : 4,
                "position_resting" : 5,
                "position_polishing" : 6,
                "stay_polishing" : 7 # Stay with ee-vel
}

N_JOINTS = 6



class MoveWheel():
    def __init__(self, setUp=""):
        # Initialize variables -- currently constant obstacle geometry
        rospy.init_node("talker_robotWheel", anonymous=True)

        self.mutex = Lock()

        # self.pub_orient = rospy.Publisher("/lwr/joint_controllers/passive_ds_command_orient", Quaternion, queue_size=10)
        # self.pub_cartVel = rospy.Publisher("/ur5/ur5_cartesian_velocity_controller_sim/command_cart_vel", cartesian_state_msg.PoseTwist, queue_size=10)
        # self.pub_cartVel = rospy.Publisher("/ur5/ur5_cartesian_velocity_controller_sim/command_cart_vel", Twist, queue_size=10)
        self.pub_jointVel = rospy.Publisher("/ur5/ur_driver/joint_speed", JointTrajectory, queue_size=5)
        
        # First subscriber call back variable
        self.recieved_eeVel_msg = False
        self.recieved_jointState_msg = False

        # self.active_controlMode = "passive_ds"
        self.active_controlMode = "position_polishing"

        # Initialize topics
        self.sub_eeVel = rospy.Subscriber("/ur5/tool_velocity", TwistStamped, self.callback_eeVel)
        self.sub_joint_pos = rospy.Subscriber("/ur5/joint_states", JointState, self.callback_jointState)

        self.freq = 50
        self.dt = 1./self.freq
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        # self.jointPos_polishing = np.array([-46, -65, -76, -177, -90, -66])/180.*pi
        # self.jointPos_polishing = np.array([-0, -58, -60, -142, -40, -16])/180.*pi
        # self.jointPos_polishing = np.array([-1.0495436827289026, -1.0166247526751917, -0.004655186329976857, -2.4832008520709437, -0.6891639868365687, -0.2714450995074671])
        # self.jointPos_polishing = np.array([-1.2447784582721155, -1.0794275442706507, 1.4450215101242065, -1.6845157782184046, -1.509059254323141, -1.2022836844073694])
        # self.jointPos_polishing = np.array([-0.9789636770831507, -0.8702791372882288, 2.390969753265381, -3.6975944677936, -1.3513978163348597, 0.24572904407978058])
        self.jointPos_polishing = np.array([-2.2322571913348597, -0.4908507505999964, 1.9028863906860352, -1.998406712208883, -1.5781844297992151, 0.10208088159561157])

        self.jointPos_resting = np.array([-94, -18, -138, -116, -90, 66])/180.*pi

        while not (self.recieved_eeVel_msg and
                   self.recieved_jointState_msg):
            rospy.sleep(0.5)
            print("Waiting for first callbacks...")
            if rospy.is_shutdown():
                print('Shuting down')
                break

        # Create publisher
        self.it_attr = 0     # only for linear 

    def run(self):
        
        self.itCount = 0
        print('Start wheel movement.')
            
        # Entering UPDATE loop
        while not rospy.is_shutdown():
            if not (self.active_controlMode in controlModes):
                warnings.warn("Wrong control mode")

            if controlModes[self.active_controlMode] == controlModes["position_polishing"]:
                if not(self.itCount%10):
                    print('update_jointPos')
                self.update_jointPos(self.jointPos_polishing)
                # self.update_jointPos(self.jointPos_resting)
                
            # if controlModes[self.active_controlMode] == controlModes["gravity_comp"]:
                # self.pub_gravity_comp.publish(std_msg.Bool(True))
                
            self.rate.sleep()  # Wait zzzz*

            if not(self.itCount % 100):
                print('Loop #{}'.format(self.itCount))

                # if controlModes[self.active_controlMode] == controlModes["passive_ds"]:
                    # print('vel des:', self.vel_des)
                    # print('angvel des:', ang_vel)
                    # print('norm vel des:', LA.norm(self.vel_des) )
                    # print('pos:', self.position_robo)
                
            self.itCount += 1

        # self.shutdown_command()
        

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

        # print('Null message send')

        # Publish several time // TODO closed loop
        # for ii in range(10):
            # zeroVel = Twist()
            # print(tuple(self.vel_des))
            # newVel.angular = Vector3(tuple(self.vel_des))
            # zeroVel.linear = Vector3(0, 0, 0)
            # zeroVel.angular = Vector3(0, 0, 0)
        
            # self.pub_vel.publish(zeroVel)
            # rospy.sleep(0.1)

    
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

            # TODO -- directly publish to robot velocity 
            # des_new_pos = self.joint_pos + des_joint_vel*(self.dt*time_fac)

            # msg_jointPos = std_msg.Float64MultiArray()
            # msg_jointPos.data = des_new_pos.tolist()

    def get_distance_from_attractor(self):
        return LA.norm((self.attractors[:, self.it_attr] - self.position_robo))
    
    
    def compute_ds_linearAttractor_const(self, velConst=0.3, distSlow=0.2):
        self.attractors = np.array([[0, 0.0, 1.16],
                                    [0, 0.5, 0.5],
                                    [0.5, 0., 0.5],
                                    [0.0, -0.5, 0.5]]).T
        
        if (self.get_distance_from_attractor() < 0.20): # Adapting attractor
            self.it_attr = (self.it_attr + 1) % self.attractors.shape[1]
        
        vel_des = (self.attractors[:, self.it_attr] - self.position_robo)

        dist_attr = LA.norm(vel_des)

        vel_des = np.min([dist_attr, distSlow])*velConst/distSlow*vel_des/dist_attr

        return vel_des

    def compute_ds_circular_motion(self, rad_circle=0.2, **kwargs):
        self.compute_ds_circular_motion(axis_ellipse=np.ones(2)*rad_circle, **kwargs)

    def compute_ds_ellipsoid_motion(self, center_pos=[0.2, 0.0, 0.9], beta=1, sigma=10, zeta=1, vel_const=0.2, axis_ellipse=[0.2,0.1], vel_mag=0.4, pow_elli=2):
        # TODO rotation?
        pos_rel = self.position_robo - np.array(center_pos)
        pos_rel[1:] = 1./np.array(axis_ellipse) * pos_rel[1:]
        
        normPos_yz = LA.norm(pos_rel[1:])
        # normPos_yz = np.sqrt(np.sum((pos_rel[1:]/np.array(axis_ellipse))**pow_elli))
        if normPos_yz:
            delta_rad = (normPos_yz - 1)/normPos_yz

        vel_des = np.zeros(3)
        vel_des[0] = - pos_rel[0]
        vel_des[1] = - pos_rel[2] - sigma*pos_rel[1]*delta_rad
        vel_des[2] =  pos_rel[1] - sigma*pos_rel[2]*delta_rad

        vel_des[1:] = np.array(axis_ellipse)*vel_des[1:]

        vel_norm = LA.norm(vel_des)
        if vel_norm:
            vel_des = vel_des/vel_norm

        return vel_des * vel_mag

    # def compute_des_quat(self, )
        # print('not implemented')
    def callback_eeVel(self, msgs):
        if not self.recieved_eeVel_msg:
            self.recieved_eeVel_msg = True
            print("Recieved first ee velocity")
        
    def callback_pose(self, msg):
        self.position_robo =  np.array([msg.position.x,
                                        msg.position.y,
                                        msg.position.z])
        
        self.quat_robo = np.array([msg.orientation.w,
                                   msg.orientation.x,
                                   msg.orientation.y,
                                   msg.orientation.z])
        
        self.recieved_pose_msg = True
        print("Recieved first joint state")

    def callback_jointState(self, msg):
        with self.mutex:
            self.joint_pos = np.array(msg.position[:N_JOINTS])
            self.joint_vel = msg.velocity[:N_JOINTS]
            self.joint_torque = msg.effort[:N_JOINTS]

            if not self.recieved_jointState_msg:
                self.recieved_jointState_msg = True
                print("Recieved first joint state")


    def callback_controlMode(self, msg):
        if msg.data in controlModes:
            print("Switched from control mode <<{}>> to <<{}>>".format(self.active_controlMode, msg.data))
            self.active_controlMode = msg.data
        else:
            print("Suggested control <<{}>> does not exist.".format(msg.data))
            print("Possible control modes are:")
            print(controlModes.keys())
            print("\nWe stay in <<{}>>-mode.".format(self.active_controlMode))
    
if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)
        
        MoveWheel_Instance = MoveWheel('conveyerBelt_basket')
        
        signal.signal(signal.SIGINT, MoveWheel_Instance.shutdown_command)

        if not rospy.is_shutdown():
            MoveWheel_Instance.run()
        
    except rospy.ROSInterruptException:
        pass
