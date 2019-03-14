#!/usr/bin/env python2
import numpy as np
import numpy.linalg as LA
import warnings

import matplotlib.pyplot as plt

import json

import os
os.chdir("/home/lukas/catkin_ws/src/wheel_polishing/scripts/")

from testLines import calculate_cuver

N_JOINTS = 6 # == DIM
N_JOINTS = 1 # == DIM

# DT = 1./25 # fixed DT
DT = 0.04 # make sure this is bigger than self.dt

# TODO -- regression algorithm

class InterpolatePoints():
    def __init__(self, n_iteration_points=10, n_loops=-1):
        self.n_iteration_points = n_iteration_points
        self.n_loops = n_loops # number times the points are looped (-1==inf)
        
        self.vel_boundary = np.zeros((N_JOINTS, 2))
        self.pos_boundary = np.zeros((N_JOINTS, 2))
        self.dt_interval = np.ones(2) * DT

        self.data_points = []
        with open('../data/ridgeback_calibration_v2.json') as json_data:
            self.data_points = json.load(json_data)

        self.n_points=10 # maximum number of points

        print('Class initialized')

    def run(self):
        self.dt = 0.01
        self.time = 0
        attr_margin = 0.001 # set margin

        self.time_attr = self.time

        self.positions = np.zeros((N_JOINTS, 2))
        self.t_loop = [0, self.dt]
        
        self.it_loop = 1

        goal_attr_reached = False

        positions_attr = np.zeros((N_JOINTS, self.n_points))
        velocities_attr = np.zeros((N_JOINTS, self.n_points))
        time_attr = np.zeros(self.n_points)

        self.spline_factors = np.zeros((N_JOINTS, 4)) # spline 4 order
        
        self.it_attr = 0

        self.callback_pos()

        plt.subplots()

        self.time += self.dt

        while True:
            # TODO update spline each loop to have flexible DS
            # print('update spline')
            self.update_spline()
            # print('attr', self.it_attr)

            vel = self.get_interpolated_velocity()

            self.positions = np.vstack((self.positions.T, self.joint_pos + vel*self.dt)).T
            t_loop = self.time
            self.t_loop.append(t_loop)

            
            if self.get_distance_from_attractor() < attr_margin:
                if goal_attr_reached:
                    print('Movement is finished.')
                    break
                
                positions_attr[:, self.it_attr] = self.pos_boundary[:,1]
                velocities_attr[:, self.it_attr] = self.vel_boundary[:,1]
                time_attr[self.it_attr] = self.time
                
                self.it_attr += 1
                self.time_attr = self.time

                goal_attr_reached = self.update_boundary_conditions()
                
                # for ii in range(N_JOINTS):
                    # plt.subplot(N_JOINTS, 1, ii+1)
                    # plt.plot(time_attr[self.it_attr], positions_attr[ii,self.it_attr], 'ro')

            self.callback_pos()
            self.time += self.dt
            self.it_loop += 1

            # calculate_cuver(DT, self.spline_factors, self.pos_boundary,self.vel_boundary, np.max([self.time_attr-self.time,0])+DT, t_offset=self.time)
            
            
            # import pdb; pdb.set_trace() # set breakpoint

            
            if not(self.it_loop%10):
                print('Loop #', self.it_loop)

            if self.it_loop > 200:
                print('Maximum loop number reached')
                break

            # t_loop = np.arange(self.it_loop+1)*self.dt
            # t_attr = np.arange(self.it_attr)*DT

            # plt.subplots(N_JOINTS, 1, 1)
            # for ii in range(N_JOINTS):
                # plt.subplot(N_JOINTS, 1, ii+1)
                # plt.plot(t_attr, positions_attr[ii,:self.it_attr], 'ro')
                # plt.plot(t_loop, self.positions[ii,:], '.')

                # plt.plot(t_loop, self.positions[ii,-1], 'k.')

        for ii in range(N_JOINTS):
            plt.subplot(N_JOINTS, 1, ii+1)
            plt.plot(self.t_loop, self.positions[ii,:], 'k.')
            for jj in range(positions_attr.shape[1]):
                plt.plot(time_attr, positions_attr[ii,:], 'ro')
                dt_arrow = 0.1
                plt.plot([time_attr[jj], time_attr[jj]+dt_arrow],[positions_attr[ii,jj], positions_attr[ii, jj]+velocities_attr[ii, jj]*dt_arrow], 'r--')
                
            # plt.plot(t_attr, positions_attr[ii,:self.it_attr], 'ro')
            # plt.plot(t_attr, positions_attr[ii,:self.it_attr], 'ro')
            
        plt.ion()
        plt.show()
        

    def update_spline(self):
        # Create a curve  of the form spline approximation ORDER == 4
        # x(t) = c_0 + c_1*t + + c_2*t^2 + + c_3*t^3
        self.pos_boundary[:, 0] = self.joint_pos
        self.vel_boundary[:, 0] = self.joint_vel

        self.spline_factors[:, 0] = self.pos_boundary[:, 0]
        self.spline_factors[:, 1] = self.vel_boundary[:, 0]

        # Time to next goal
        # print('goal time', self.time_attr+DT)
        # print('time now', self.time)
        dt = np.max([self.time_attr+DT-self.time, self.dt*1.0])
        c_matr = np.array(([[dt**2, dt**3],
                            [2*dt, 3*dt**2]]))
        c_matr_inv = LA.inv(c_matr)
        
        for ii in range(N_JOINTS):
            c_vect = np.array(([self.pos_boundary[ii, 1]-self.spline_factors[ii,0]-self.spline_factors[ii,1]*dt, self.vel_boundary[ii, 1]-self.spline_factors[ii,1]]))

            c_vect = c_matr_inv.dot(c_vect)
            self.spline_factors[ii,2] = c_vect[0]
            self.spline_factors[ii,3] = c_vect[1]

    
    def get_interpolated_velocity(self, RK4=True):
        dt = self.dt
        # dt = self.time - self.time_attr
        
        pos_next = (self.spline_factors[:,0] + self.spline_factors[:,1]*dt
                    + self.spline_factors[:,2]*dt**2+ self.spline_factors[:,3]*dt**3)

        vel = (pos_next - self.pos_boundary[:, 0]) / dt
        
        # import pdb; pdb.set_trace()
        # print('pos_boundary'); print(np.round(self.pos_boundary, 2))
        # print('vel_boundary'); print(np.round(self.vel_boundary, 2))

        # TODO RK4 interpolation
        # TODO LIMIT maximum speed
        
        return vel

    def get_distance_from_attractor(self):
        delta_pos =  self.pos_boundary[:, 1] - self.joint_pos
        return LA.norm((delta_pos))
        # TODO Get distance only points which are on the 'wrong' side
        # ind_wrong = (self.pos_boundary[:, 2]-self.joint_pos)*delta_pos < 0
        # return LA.norm((delta_pos[ind_wrong]))

    def update_boundary_conditions(self):
        # return (normal=0) -- (shutdown=1)
        if (self.it_attr+1 >= self.n_points):
            if self.n_loops == 0:
                print('Shutdown')
                self.vel_boundary[:, 1] = np.zeros(N_JOINTS)
                self.pos_boundary[:, 1] = self.pos_boundary[:, 0] 
                return 1 # shutdown 
            else:
                if self.n_loops > 0:
                    self.n_loops -= 1
                self.it_attr = 0

        if type(self.data_points[self.it_attr]) == dict:
            self.pos_boundary[:, 1] = self.data_points[self.it_attr]['position'][:N_JOINTS]
            self.vel_boundary[:, 1] = self.data_points[self.it_attr]['velocity'][:N_JOINTS]
        # elif type(self.data_points[self.it_attr]) == dict:
        
        # Calculate dt based on position 
        # self.vel_boundary[:, 1] = (self.pos_boundary[:, 2]-self.pos_boundary[:, 1])/dt
        # ind_directionChange = (self.vel_boundary[:,0] * self.vel_boundary[:,1])
        # self.vel_boundary[ind_directionChange, 1] = 0
        
        # TODO remove intermediary variable
        return 0 # continue loops

    def callback_pos(self):
        self.joint_pos = self.positions[:,-1]
        self.joint_vel = (self.positions[:,-1]-self.positions[:,-2])/self.dt
        self.time = self.time + self.dt

def __main__():
    InterpolatePoints_isnt = InterpolatePoints()
    InterpolatePoints_isnt.run()
    
    return 0

__main__()
