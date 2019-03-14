#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt

def calculate_cuver(DT, spline_factors, pos_boundary, t_offset=0):
    DIM=spline_factors.shape[0]
    
    n_points = 30
    pos_sample = np.zeros((DIM, n_points))
    t_sample = np.zeros((n_points))

    for ii in range(n_points):
        t = ii*DT - n_points/3.0*DT
        pos_next = (spline_factors[:,0] + spline_factors[:,1]*t
                        + spline_factors[:,2]*t**2+ spline_factors[:,3]*t**3)

        t_sample[ii] = t
        pos_sample[:,ii] = pos_next

    plt.figure()
    plt.plot(t_sample + t_offset, pos_sample[0,:])
    plt.ion()
    plt.show()
    # vel = (pos_next - pos_boundary[:, 0])/dt


def __main__():
    pos_boundary = np.array([[0, 1]])
    vel_boundary = np.array([[0, 0]])

    spline_factors = np.zeros((1,4))

    time_next_attr = 1
    time = 0
    DT = 0.1

    N_JOINTS = 1

    spline_factors[:, 0] = pos_boundary[:, 0]
    spline_factors[:, 1] = vel_boundary[:, 0]

    # import pdb; pdb.set_trace()
    # Time to next goal
    dt = np.max([time_next_attr - time, DT*1.0])

    c_matr_inv = LA.inv(np.array(([[dt**2, dt**3],
                                   [2*dt, 3*dt**2]])))

    for ii in range(N_JOINTS):
        c_vect = np.array(([pos_boundary[ii, 1]-spline_factors[ii,0]-spline_factors[ii,1]*dt, vel_boundary[ii, 1]-spline_factors[ii,1]]))

        c_vect = c_matr_inv.dot(c_vect)
        spline_factors[ii,2] = c_vect[0]
        spline_factors[ii,3] = c_vect[1]

    calculate_cuver(DT, spline_factors, pos_boundary)

__main__()
