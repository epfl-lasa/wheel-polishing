#!/usr/bin/env python2

import numpy as np
import numpy.linalg as LA
import warnings

import matplotlib.pyplot as plt

import json

N_JOINTS = 6 # == DIM
# DIM = 2
DT = 1./25 # fixed DT

# TODO -- regression algorithm


class InterpolatePoints():
    def __init__(self, n_iteration_points=10, n_loops=0):
        self.n_iteration_points = n_iteration_points
        self.n_loops = n_loops # number times the points are looped (-1==inf)
        
        self.vel_boundary = np.zeros((N_JOINTS, 2))
        self.pos_boundary = np.zeros((N_JOINTS, 3))
        self.dt_interval = np.ones(2) * DT

        self.data_points = []
        with open('../data/ridgeback_calibration_v2.json') as json_data:
            self.data_points = json.load(json_data)

        # import pdb; pdb.set_trace() # set breakpoint
        
        # self.get_velocity_future_boundary()

        self.n_points=10 # maximum number of points

        print('Class initialized')

    def run(self):
        # plt.figure()
        # plt.plot()
        # plt.show()

        self.dt = 0.01
        self.time = 0
        attr_margin = 0.1 # set margin

        self.positions = np.zeros((N_JOINTS, 2))
        self.it_loop = 1

        goal_attr_reached = False

        positions_attr = np.zeros((N_JOINTS, self.n_points))

        self.spline_factors = np.zeros((N_JOINTS, 4)) # spline 4 order
        
        self.it_attr = 0
        while True:
            self.callback_pos()

            import pdb; pdb.set_trace() ## DEBUG ##
            
            if self.get_distance_from_attractor() < attr_margin:
                if goal_attr_reached:
                    break 
                self.it_attr += 1
                goal_attr_reached = self.update_boundary_conditions()

                print('it loop', self.it_loop)
                positions_attr[:, self.it_attr] = self.pos_boundary[:,1]
                
            vel = self.get_interpolated_velocity()

            self.positions = np.vstack((self.positions.T, self.positions[:,self.it_loop-1] + vel*DT)).T
            
            self.time += self.dt

            self.it_loop += 1
            
            if not(self.it_loop%100):
                print('Loop #', self.it_loop)
            
            if self.it_loop > 10:
                break

        # import pdb; pdb.set_trace() # set breakpoint
        
        t_loop = np.arange(self.it_loop+1)*self.dt
        t_attr = np.arange(self.it_attr)*DT

        # plt.subplots(N_JOINTS, 1, 1)
        plt.subplots()
        
        for ii in range(N_JOINTS):
            plt.subplot(N_JOINTS, 1, ii+1)
            plt.plot(t_attr, positions_attr[ii,:self.it_attr], 'ro')
            plt.plot(t_loop, self.positions[ii,:], '.')

        plt.ion()
        plt.show()
        import pdb; pdb.set_trace() ## DEBUG ##
        
    def get_interpolated_velocity(self):
        dt = self.time - self.t0
        vel = (self.spline_factors[:,0] + self.spline_factors[:,1]*dt
               + self.spline_factors[:,2]*dt**2+ self.spline_factors[:,3]*dt**3)
        
        # LIMIT maximum speed
        return vel

    def get_distance_from_attractor(self):
        delta_pos =  self.pos_boundary[:, 1] - self.joint_pos
        return LA.norm((delta_pos))
        # Get distance only points which are on the 'wrong' side
        # ind_wrong = (self.pos_boundary[:, 2]-self.joint_pos)*delta_pos < 0
        # return LA.norm((delta_pos[ind_wrong]))

    def update_boundary_conditions(self):
        # return (normal=0) -- (shutdown=1)
        self.t0 = self.time

        self.pos_boundary[:, 0] = self.joint_pos
        
        self.vel_boundary[:, 0] = self.joint_vel

        self.pos_boundary[:, 1] = self.pos_boundary[:,2]


        if (self.it_attr+1 >= self.n_points):
            if self.n_loops == 0:
                print('Shutdown')
                self.vel_boundary[:, 1] = np.zeros(N_JOINTS)
                self.pos_boundary[:, 2] = self.pos_boundary[:, 1]
                return 1 # shutdown 
            else:
                if self.n_loops > 0:
                    self.n_loops -= 1
                self.it_attr = 0
                
        self.pos_boundary[:, 2] = self.data_points[self.it_attr]['position']
        
        # Calculate dt based on position 
        dt = DT
        self.vel_boundary[:, 1] = (self.pos_boundary[:, 2]-self.pos_boundary[:, 1])/dt
        ind_directionChange = (self.vel_boundary[:,0] * self.vel_boundary[:,1])
        self.vel_boundary[ind_directionChange, 1] = 0
        
        
        # spline approximation ORDER == 4
        # x(t) = c_0 + c_1*t + + c_2*t^2 + + c_3*t^3
        # TODO remove intermediary variable
        self.spline_factors[:, 0] = self.pos_boundary[:, 0]
        self.spline_factors[:, 1] = self.vel_boundary[:, 0]

        c_matr_inv = LA.pinv(np.array(([[dt**2, dt**3],
                                        [2*dt, 3*dt**2]])))
                          
        for ii in range(N_JOINTS):
            c_vect = np.array(([self.pos_boundary[ii, 1]-self.spline_factors[ii,0]-self.spline_factors[ii,1]*dt,
                                self.vel_boundary[ii, 1]-self.spline_factors[ii,1]]))

            c_vect = c_matr_inv.dot(c_vect)
            self.spline_factors[ii,2] = c_vect[0]
            self.spline_factors[ii,3] = c_vect[1]
                
        return 0 # continue loops

    def callback_pos(self):
        self.joint_pos = self.positions[:,self.it_loop]
        self.joint_vel = (self.positions[:,self.it_loop]-self.positions[:,self.it_loop-1])/self.dt
        self.time = self.time + self.dt

def __main__():
    InterpolatePoints_isnt = InterpolatePoints()
    InterpolatePoints_isnt.run()
    
    return 0


__main__()
