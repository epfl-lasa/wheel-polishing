#!/usr/bin/env python2

'''
Movement of the wheel fro the r5

@author lukashuber
@date 2019-03-01
'''

import rospy
import rospkg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped, Twist, Pose, PoseStamped, Quaternion, Point, PointStamped

from nav_msgs.msg import Path

import numpy as np
import numpy.linalg as LA
import warnings

from os.path import join

import json

import warnings
import sys
import signal
import os # make beep sound for debugging

from multiprocessing import Lock

N_JOINTS = 6 # == DIM
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


# DT = 1./25 # fixed DT
# TODO get frequency! at real time
DT_INPUT = 0.02 # make sure this is bigger than self.dt_pub

# TODO -- regression algorithm

class MoveWheelTrajectory():
    def __init__(self, setUp="", n_loops=5, smoothen_velocity=True, time_between_pointsr=1):
        # n_loops: negative number account result in infinite loops
        self.mutex = Lock()
        self.rospack = rospkg.RosPack()
        rospy.init_node("talker_playBack", anonymous=True)
        self.time_start = rospy.get_time()
        
        self.pub_jointVel = rospy.Publisher("/ur5/ur_driver/joint_speed", JointTrajectory, queue_size=5)
        self.pub_jointTraj = rospy.Publisher("/debugging/joint_trajectory", Path, queue_size=5)
        self.pub_jointTraj2 = rospy.Publisher("/debugging/joint_trajectory2", Path, queue_size=5)

        self.pub_point = []
        for ii in range(8):
            self.pub_point.append(rospy.Publisher("/debugging/poin{}".format(ii), PointStamped, queue_size=5))

        # First subscriber call back variable
        self.recieved_eeVel_msg = False
        self.recieved_jointState_msg = False

        # Initialize topics
        self.sub_joint_pos = rospy.Subscriber("/ur5/joint_states", JointState, self.callback_jointState)

        # self.freq = 25 # Warning - too high frequence leads to jerky behavior
        # self.freq = 125
        self.freq = 70
        # self.freq = 50
        # self.freq = 20
        # self.freq = 10

        self.dt_pub = 1./self.freq
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        self.n_loops = n_loops # number times the points are looped (-1==inf)
        self.smoothen_velocity = smoothen_velocity
        self.time_between_pointsr = time_between_pointsr

        print('Node initialized')

    
    def run(self):
        self.attr_margin = 0.05 # set margin
        # self.time_attr = rospy.get_time()
        # import pdb; pdb.set_trace()
        self.arriving_time_attr = rospy.get_time() + self.time_between_pointsr
        goal_attr_reached = False
        self.spline_factors = np.zeros((N_JOINTS, 4)) # spline 4 order

        while not (self.recieved_jointState_msg):
            rospy.sleep(0.5)
            print("Waiting for first callbacks...")
            if rospy.is_shutdown():
                print('Shutting down')
                break

        self.it_attr = 0 
        self.attr_step = 1

        self.data_points=[]
        # self.define_trajectory_points("play_back")
        # self.define_trajectory_points("home_polish_oscillation")
        # self.define_trajectory_points("go_home")
        # self.define_trajectory_points("go_polish")
        # self.define_trajectory_points("standing_up")
        # self.define_trajectory_points("random_movement")
        
        # Usefull ones
        # self.define_trajectory_points("leave_home")
        self.define_trajectory_points("tilt_towards_robot")

        self.n_points=len(self.data_points) # maximum number of points

        # TODO more general, to include more points in spline
        self.pos_boundary = np.tile(self.joint_pos, (2,1)).T
        self.vel_boundary = np.zeros((N_JOINTS, 2))
        
        self.it_count = 0

        self.update_boundary_conditions()
        
        self.old_time = rospy.get_time()
        while not rospy.is_shutdown():
            new_time = rospy.get_time()
            rel_time_margin = 0.1
            if (new_time - self.old_time) > self.dt_pub*(1+rel_time_margin):
                print('')
                print('WARNING - slop update', new_time - self.old_time)
                # import pdb; pdb.set_trace()
                
            self.old_time = new_time

            # TODO update spline each loop to have flexible DS
            self.update_spline()
            self.update_velocity()
            # self.publish_path_debugging()

            if self.check_if_attractor_reached():
                # os.system('play -nq -t alsa synth 0.1 sine 550')
                if goal_attr_reached:
                    print('Movement is finished.')
                    self.shutdown_command()
                    break
                self.check_if_multiple_attractor_reached()

                print('');print('');print('')
                print('Reached attractor #{} of {}'.format(self.it_attr+1, self.n_points))
                print("At time={}s".format(rospy.get_time()-self.time_start))
                print("Des Vel")
                print(self.vel_boundary[:, 1])
                print('');print('');print('')
                if self.it_attr+1==2:
                    time_real = rospy.get_time()
                    # import pdb; pdb.set_trace()

                self.it_attr += self.attr_step
                self.arriving_time_attr = rospy.get_time() + self.time_between_pointsr

                goal_attr_reached = self.update_boundary_conditions()
            # else:
                # print('Nothing reached loop')
                
            self.it_count += 1

            if not(self.it_count%100):
                print('Iteration #{}'.format(self.it_count) )

            self.rate.sleep()
            

    def publish_path_debugging(self, n_pub=100):
        # import pdb; pdb.set_trace()
        dt = np.max([self.arriving_time_attr-rospy.get_time(), self.dt_pub*1.0])
        print('dt attr', dt)
        q1 = 0
        q2 = 1
        qq = [q1, q2]

        delta_t = dt/(n_pub/2)
        
        msg_path = Path()
        msg_path.header.stamp = rospy.Time.now()
        msg_path.header.frame_id = "base_link"

        msg_path2 = Path()
        msg_path2.header = msg_path.header

        # pos = t,q1,q2
        pos0 = self.spline_factors[:,0]
        delta_axis = np.array([0,-1,-1])
        delta_pos = delta_axis - np.array([0,pos0[q1], pos0[q2]])
        
        points = np.zeros((3, n_pub))

        for ii in range(n_pub):
            dt = delta_t*ii - delta_t*int(n_pub*0.3)
            pos_next = (self.spline_factors[:,0] + self.spline_factors[:,1]*dt
                        + self.spline_factors[:,2]*dt**2+ self.spline_factors[:,3]*dt**3)
            pos = np.array(([dt, pos_next[q1], pos_next[q2]])) + delta_pos
            newPose = PoseStamped()
            newPose.pose.position = Point(pos[0], pos[1], 0)
            newPose.pose.orientation = Quaternion(0,0,0,1) # unit quaternion
            msg_path.poses.append(newPose)

            newPose = PoseStamped()
            newPose.pose.position = Point(pos[0], pos[2], 0)
            newPose.pose.orientation = Quaternion(0,0,0,1) # unit quaternion
            msg_path2.poses.append(newPose)

        self.pub_jointTraj.publish(msg_path)
        self.pub_jointTraj2.publish(msg_path2)

        for ii in range(2):
            newPoint = PointStamped()
            newPoint.header = msg_path.header
            newPoint.point = Point(dt*ii, self.pos_boundary[q1,ii]+delta_pos[1], 0)
            self.pub_point[ii+0].publish(newPoint)

            newPoint = PointStamped()
            newPoint.header = msg_path.header
            newPoint.point = Point(dt*ii, self.pos_boundary[q2,ii]+delta_pos[2], 0)
            self.pub_point[ii+2].publish(newPoint)

            
        ii_start = 4
        for ii in range(2):
            newPoint = PointStamped()
            newPoint.header = msg_path.header
            dt_vel = 1*self.dt_pub
            newPoint.point = Point(dt*1+dt_vel, self.pos_boundary[qq[ii],1]+self.vel_boundary[qq[ii],1]*dt_vel+delta_pos[ii+1], 0)
            self.pub_point[ii+ii_start].publish(newPoint)

        ii_start = 6
        for ii in range(2):
            # Velocity point
            newPoint = PointStamped()
            newPoint.header = msg_path.header
            newPoint.point = Point(self.dt_pub, self.dt_pub*self.des_joint_vel_debug[qq[ii]]+delta_axis[1+ii],0)
            self.pub_point[ii+ii_start].publish(newPoint)
        
        
        print('vel boundary {}'.format(self.it_count) )
        print(self.joint_vel)
        print('vel command {}'.format(self.it_count) )
        print(self.des_joint_vel_debug)

        self.vel_command_old_debug = self.des_joint_vel_debug
        
    
            
        
    def shutdown_command(self, signal=0, frame=0):
        # TODO activate after CTRL-c
        print('Shuting down....')

        msg_jointVel = JointTrajectory()
        msg_jointVel.header.stamp = rospy.Time.now()

        msg_jointVel.joint_names = JOINT_NAMES
        # msg_jointVel.points.velocities = des_joint_vel.tolist()

        newPoint = JointTrajectoryPoint()
        newPoint.positions = self.joint_pos.tolist()
        newPoint.velocities = np.zeros(N_JOINTS).tolist()
        for ii in range(3): # repeat several times
            msg_jointVel.points.append(newPoint)

        self.pub_jointVel.publish(msg_jointVel)
        print('Send zero velocity')
        
        rospy.signal_shutdown('User shutdown.')
        print('See you next time')
        
    def update_spline(self):
        # Create a curve  of the form spline approximation ORDER == 4
        # x(t) = c_0 + c_1*t + + c_2*t^2 + + c_3*t^3
        self.pos_boundary[:, 0] = self.joint_pos
        self.vel_boundary[:, 0] = self.joint_vel

        self.spline_factors[:, 0] = self.pos_boundary[:, 0]
        self.spline_factors[:, 1] = self.vel_boundary[:, 0]
        # import pdb; pdb.set_trace()

        # Time to next goal
        dt = np.max([self.arriving_time_attr-rospy.get_time(), self.dt_pub*1.0])
        c_matr = np.array(([[dt**2, dt**3],
                            [2*dt, 3*dt**2]]))
        c_matr_inv = LA.inv(c_matr)
        
        for ii in range(N_JOINTS):
            # TODO parallel caclulation for speed
            c_vect = np.array(([self.pos_boundary[ii, 1]-self.spline_factors[ii,0]-self.spline_factors[ii,1]*dt, self.vel_boundary[ii, 1]-self.spline_factors[ii,1]]))

            c_vect = c_matr_inv.dot(c_vect)
            self.spline_factors[ii,2] = c_vect[0]
            self.spline_factors[ii,3] = c_vect[1]

    def update_velocity(self, vel_limit=0.3, acc_fac=1.0, slow_down_time=0.1, uniform_scaling=True):
        des_joint_vel = self.get_interpolated_velocity()

        # print('self.joint_pos', 'joint_pos1', 'joint_pos1')
        # print(np.vstack((self.joint_pos[:dd], self.pos_boundary[:dd].T)).T)

        max_des_vel = np.abs(des_joint_vel)
        # print('max vel', np.max(np.abs(des_joint_vel)) )
        # print('ratio vel', np.min([vel_limit/max_des_vel, 1]) )

        # des_joint_vel_init = des_joint_vel*1.0
        
        if any(vel_limit/max_des_vel < 1):
            print("reducing vel max")
            if (self.arriving_time_attr - rospy.get_time()) < slow_down_time:
                self.arriving_time_attr = slow_down_time + rospy.get_time()
            else:
                self.arriving_time_attr += self.dt_pub
            
            # if uniform_scaling:
            des_joint_vel = vel_limit/np.max(max_des_vel)*des_joint_vel
            # else:
                # import pdb; pdb.set_trace()
                # des_joint_vel = np.min(np.vstack((vel_limit/max_des_vel, np.ones(N_JOINTS))), axis=0)*des_joint_vel
            
        self.des_joint_vel_debug = des_joint_vel

        # TODO - PID controller in "virtual velocity" -- ?

        # print('des_joint_vel_ini','des_joint_vel', 'vel_boundary 1', 'vel_boundary 1')
        # print(np.vstack((des_joint_vel_init[:dd], des_joint_vel[:dd], self.vel_boundary[:dd].T)).T)
        # print('des_joint_vel', 'vel_boundary 1', 'vel_boundary 1')
        # print(np.vstack(( des_joint_vel[:dd], self.vel_boundary[:dd].T)).T)
        # print('spline')
        # print(self.spline_factors[:2,:])

        # print('des_joint_vel_mag', LA.norm(des_joint_vel))
        # des_joint_vel = des_joint_vel # TODO REMOVE SAFETY RATIO

        # LIMIT RELATIVE ACCELERATION
        # print('des_joint_vel', des_joint_vel)
        # delta_vel = (des_joint_vel- self.joint_vel)
        # acc_ratio = 1./np.array([4,3,2,1,1,1])*acc_fac
        # acc_des = np.abs(delta_vel/self.dt_pub)
        # if any(acc_des > acc_ratio):
            # max_ind = np.argmax(acc_des/acc_ratio)
            # print(max_ind)
            # des_joint_vel_corr = acc_ratio[max_ind]/acc_des[max_ind]*delta_vel[max_ind] + self.joint_vel[max_ind]
            # print('delta_vel', delta_vel)
            # print('delta vel', (des_joint_vel[max_ind] - des_joint_vel_corr)/des_joint_vel[max_ind])
            # max_rat = des_joint_vel_corr/des_joint_vel[max_ind]
            # print(
            # max_rat = np.max([np.min([des_joint_vel_corr/des_joint_vel[max_ind], 1]), 0.1])
            # print('des joint vel', des_joint_vel_corr)
            # print('max_rat', max_rat)

            # des_joint_vel = max_rat*des_joint_vel
            # print('vel rat', acc_ratio[max_ind]/acc_des[max_ind])
            # des_joint_vel = acc_ratio[max_ind]/acc_des[max_ind]*delta_vel + self.joint_vel
            # print('des_joint_vel_corr', des_joint_vel_corr)            
            # print('delta_rel', (des_joint_vel_corr-des_joint_vel)/des_joint_vel)            
            # des_joint_vel = 0.6*des_joint_vel_corr

        # Create message
        msg_jointVel = JointTrajectory()
        msg_jointVel.header.stamp = rospy.Time.now()
        msg_jointVel.joint_names = JOINT_NAMES
        newPoint = JointTrajectoryPoint()
        # newPoint.positions = (self.joint_pos + self.dt_pub*des_joint_vel).tolist()
        newPoint.velocities = des_joint_vel.tolist()

        # if not(self.it_loop%5):
            # print('des_joint', des_joint_vel)
            # print('actual_joint', self.joint_vel)
            # print('next via point', self.pos_boundary[:, 1])

        # !!! WARNING !!! The robot input message and control message have a different mapping. 
        # I.e. a mapping needs to be applied before sending commands as follows (0<->2):
        # joint mapping [ur5/joint_states -> ur5/ur_driver/joint_speed]
        # ur5_arm_shoulder_pan_joint=2 -> 0
        # 1 -> 1
        # ur5_arm_shoulder_lift_shoulder=0 -> 2
        # 3 -> 3
        # 4 -> 4
        # 5 -> 5
        # input_map  = np.array([0,1,2,3,4,5])
        # output_map = np.array([2,1,0,3,4,5])
        # msg_jointVel.joint_names[output_map] = msg_jointVel.joint_names[input_map]
        # newPoint.positions[output_map] = newPoint.positions[input_map]
        # newPoint.velocities[output_map] = newPoint.velocities[input_map]
        
        # msg_jointVel.joint_names[0], msg_jointVel.joint_names[2] = msg_jointVel.joint_names[2], msg_jointVel.joint_names[0]
        # newPoint.positions[0], newPoint.positions[2] = newPoint.positions[2], newPoint.positions[0]
        newPoint.velocities[0], newPoint.velocities[2] = newPoint.velocities[2], newPoint.velocities[0]
        # newPoint.velocities = np.zeros(N_JOINTS).tolist()
        # newPoint.velocities[5] = 0.01 # For testing
        
        msg_jointVel.points.append(newPoint)
        
        self.pub_jointVel.publish(msg_jointVel)
            
    def get_interpolated_velocity(self):
        # NOTE - We don't take the derivative directly, but estimate the velocity it takes to get to the next position
        dt = self.dt_pub

        pos_next = (self.spline_factors[:,0] + self.spline_factors[:,1]*dt
                    + self.spline_factors[:,2]*dt**2+ self.spline_factors[:,3]*dt**3)

        # vel = (pos_next - self.pos_boundary[:, 0]) / (dt)
        vel = (pos_next - self.spline_factors[:,0]) / (dt)
        # import pdb; pdb.set_trace()

        # TODO RK4 interpolation > not really, since in position.
        # TODO LIMIT maximum speed
        return vel

    def check_if_attractor_reached(self):
        # if get_distance_from_attractor(self) < self.dist_attr:
            # return False
        # else:
            # return True

        ind = ~self.attrReached
        self.attrReached[ind] = self.pos_boundary[ind,1]*self.relativeStartingDir[ind]-self.attr_margin < self.joint_pos[ind]*self.relativeStartingDir[ind]

        # Alternatively
        return all(self.attrReached)

    def check_if_multiple_attractor_reached(self):
        
        while all(self.attrReached):
            joint_pos_virt = np.array(self.data_points[self.it_attr]['position'][:N_JOINTS])
            if self.it_attr+2*self.attr_step >= self.n_points:
                break
            pos_attr = np.array(self.data_points[self.it_attr + self.attr_step]['position'][:N_JOINTS])
            self.relativeStartingDir = np.copysign(np.ones(N_JOINTS), (pos_attr-joint_pos_virt)) 
            self.attrReached = pos_attr*self.relativeStartingDir-self.attr_margin < self.joint_pos*self.relativeStartingDir

            # import pdb; pdb.set_trace()
            if all(self.attrReached):
                self.it_attr += self.attr_step
            # print('attr', self.it_attr)

    def get_distance_from_attractor(self):
        delta_pos =  self.pos_boundary[:, 1] - self.joint_pos
        return LA.norm((delta_pos))
        # TODO Get distance only points which are on the 'wrong' side
        # ind_wrong = (self.pos_boundary[:, 2]-self.joint_pos)*delta_pos < 0
        # return LA.norm((delta_pos[ind_wrong]))

    def update_boundary_conditions(self):
        # return  (normal=0) -- (shutdown=1)
        # self.arriving_time_attr = rospy.get_time() + self.time_between_pointsr
        state_shutdown = False
        # if (self.it_attr+self.attr_step >= self.n_points):
        if (self.it_attr >= self.n_points):
            if self.n_loops==0:
                print('Shutdown')
                self.vel_boundary[:, 1] = np.zeros(N_JOINTS)
                self.pos_boundary[:, 1] = self.pos_boundary[:, 0] 
                state_shutdown = True  # shutdown 
            else:
                if self.n_loops > 0:
                    self.n_loops -= 1
                    print("Number of loops left: {}".format(self.n_loops+1))
                else:
                    print("Many loops left....")
                    
                self.it_attr = 0
            

        if not state_shutdown:
            if type(self.data_points[self.it_attr]) == dict:
                self.pos_boundary[:, 1] = self.data_points[self.it_attr]['position'][:N_JOINTS]
            
            shutdown_soon = (self.it_attr+1 >= self.n_points)
            if self.smoothen_velocity and not (shutdown_soon and self.n_loops==0):
                print(""); print(""); print("");
                print('smooth vel')
                print(""); print(""); print("");
                delta_dist0 = np.subtract(self.pos_boundary[:, 1], self.joint_pos)
                delta_dist1 = np.subtract(self.data_points[(self.it_attr+1)%self.n_points]['position'][:N_JOINTS], self.pos_boundary[:, 1])
                self.vel_boundary[:, 1] = (delta_dist0 + delta_dist1)/self.time_between_pointsr
                # self.vel_boundary[:, 1] = (delta_dist1)/self.time_between_pointsr
                self.vel_boundary[:, 1] = (delta_dist1)/self.time_between_pointsr
                # import pdb; pdb.set_trace()

                # self.vel_boundary[:, 1] = (delta_dist0 + delta_dist1)/self.time_between_pointsr
                # print(delta_dist0)
                # print(delta_dist1)
                print('vel boundary', self.vel_boundary[:, 1])
            else:
                self.vel_boundary[:, 1] = self.data_points[self.it_attr]['velocity'][:N_JOINTS]
                # print("Joint vel", self.vel_boundary[:, 1])

        # print(self.it_attr)
        # Check on which side of the new attractor the robot started
        self.relativeStartingDir = np.copysign(np.ones(N_JOINTS), (self.pos_boundary[:, 1]-self.joint_pos)) 
        self.attrReached = np.zeros(N_JOINTS, dtype=bool)
        
        return state_shutdown # continue loops

    def define_trajectory_points(self, mode="go_home", time_to_go_home=0.1):
        # Time in seconds

        if mode=="play_back":
            # with open(join(rospack.get_path("robot_calibration"), "data", "ridgeback_calibration_v2.json")) as json_data:
            with open(join(self.rospack.get_path("robot_calibration"), "data", "ur5_calibration.json")) as json_data:
                self.data_points = json.load(json_data)

            self.attr_step = 5
            self.it_attr = 300
            self.time_between_pointsr = 0.02

            self.smoothen_velocity=False

            print("Got play back points")

        elif mode=="go_home":
            self.data_points = []
            with open(join(self.rospack.get_path("wheel_polishing"), "data", "ur5_home_transition_pos.json")) as json_data:
                self.data_points.append(json.load(json_data)[0])
            with open(join(self.rospack.get_path("wheel_polishing"), "data", "ur5_home_position.json")) as json_data:
                self.data_points.append(json.load(json_data)[0])

            self.data_points.append(self.data_points[1])

            print("Recieved data_points")

        elif mode=="go_polish":
            self.data_points = []
            with open(join(self.rospack.get_path("wheel_polishing"), "data", "ur5_home_transition_pos.json")) as json_data:
                self.data_points.append(json.load(json_data)[0])
            with open(join(self.rospack.get_path("wheel_polishing"), "data", "ur5_polishing_resting.json")) as json_data:
                self.data_points.append(json.load(json_data)[0])

            delta_dist0 = np.subtract(self.data_points[0]['position'][:N_JOINTS], self.joint_pos)
            delta_dist1 = np.subtract(self.data_points[1]['position'][:N_JOINTS], self.data_points[0]['position'][:N_JOINTS])

            self.data_points[0]['velocity'] = ((delta_dist1 + delta_dist0)/time_to_go_home)
            # self.data_points[1]['velocity'] = np.zeros(N_JOINTS)

            print("got data_points")

        elif mode=="home_polish_oscillation":

                self.time_between_pointsr = 10

            print("Recieved data_points")

        elif mode=="standing_up":
            with open(join(self.rospack.get_path("wheel_polishing"), "data", "standing_up.json")) as json_data:
                self.data_points = json.load(json_data)[:3]
            self.time_between_pointsr = 1

        elif mode=="random_movement":
            # with open(join(self.rospack.get_path("wheel_polishing"), "data", "record_late_night.json")) as json_data:
            # with open(join(self.rospack.get_path("wheel_polishing"), "data", "random_record.json")) as json_data:
            with open(join(self.rospack.get_path("wheel_polishing"), "data", "crazy_loop.json")) as json_data:
            # with open(join(self.rospack.get_path("crazy_loop"), "data", "random_record.json")) as json_data:
                self.data_points = json.load(json_data)[:3]
        elif mode=="leave_home":
            with open(join(self.rospack.get_path("wheel_polishing"), "data", "ur5_home_position_exiting.json")) as json_data:
                self.data_points = json.load(json_data)[:3]
        elif mode=="go_polish":
            print("TO implement")


        elif mode=="tilt_towards_robot":
            with open(join(self.rospack.get_path("wheel_polishing"), "data", "ur5_position_polishing.json")) as json_data:
                self.data_points = json.load(json_data)

            with open(join(self.rospack.get_path("wheel_polishing"), "data", "ur5_tilt_frontal.json")) as json_data:
                data = json.load(json_data)
                self.data_points = self.data_points + data + data[-2::-1]
                # self.data_points.append(data[-1])
            # import pdb; pdb.set_trace()

        else:
            print("mode not found")

    def callback_jointState(self, msg, k_pos=0.1, k_vel=0.1, k_acc=0.05):
        # TODO test noise and get standart deviation to get a elaborate kalman filter
        with self.mutex:
            if not self.recieved_jointState_msg:
                # print('header', msg.header.stamp)
                self.time_lastMsg = msg.header.stamp.to_sec()
                self.recieved_jointState_msg = True
                self.joint_pos = np.array(msg.position[:N_JOINTS])
                self.joint_vel = np.array(msg.velocity[:N_JOINTS])
                self.joint_acc = np.array(msg.velocity[:N_JOINTS])

                print("Recieved first joint state")

            new_time = msg.header.stamp.to_sec()
            dt = new_time - self.time_lastMsg

            # TODO apply transform to np.array only when used
            joint_pos_new = np.array(msg.position[:N_JOINTS])
            joint_vel_new = np.array(msg.velocity[:N_JOINTS])
            joint_acc_new = np.array(msg.effort[:N_JOINTS]) # including weight!

            # Simplified kalman filter. Improve?
            self.joint_acc = (1-k_acc)*self.joint_acc + k_acc*joint_acc_new

            self.joint_vel = (1-k_vel)*(self.joint_vel) + k_vel*joint_vel_new
            self.joint_pos = (1-k_pos)*(self.joint_pos+dt*self.joint_vel) + k_pos*joint_pos_new
            
            self.time_lastMsg = new_time



if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)
        
        MoveWheelTrajectory_instance = MoveWheelTrajectory('conveyerBelt_basket')
        signal.signal(signal.SIGINT, MoveWheelTrajectory_instance.shutdown_command)

        if not rospy.is_shutdown():
            MoveWheelTrajectory_instance.run()
        
    except rospy.ROSInterruptException:
        pass
