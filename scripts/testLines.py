#!/usr/bin/env python2
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt

def calculate_cuver(DT, spline_factors, pos_boundary,
                    vel_boundary=False, time_attr=0, t_offset=0):
    # print('DT', DT)
    # print('spline_factors', spline_factors)
    # print('pos_boundary', pos_boundary)
    # print('vel_boundary', vel_boundary)
    # print('time_attr', time_attr)
    # print('t_offset', t_offset)
    
    DIM=spline_factors.shape[0]
    
    DT = time_attr/10
    
    n_points = 30
    pos_sample = np.zeros((DIM, n_points))
    t_sample = np.zeros((n_points))

    for ii in range(n_points):
        t = ii*DT - n_points/3.0*DT
        pos_next = (spline_factors[:,0] + spline_factors[:,1]*t
                        + spline_factors[:,2]*t**2+ spline_factors[:,3]*t**3)

        t_sample[ii] = t
        pos_sample[:,ii] = pos_next

    plt.subplots()
    for ii in range(DIM):
        plt.subplot(DIM, 1, ii+1)
        plt.plot(t_sample + t_offset, pos_sample[ii,:])
        plt.plot(t_offset, pos_boundary[ii,0], 'b*')
        plt.plot(t_offset+time_attr, pos_boundary[ii,1], 'b*')
        if not type(vel_boundary)==bool:
            plt.plot([t_offset, t_offset+DT],
                     [pos_boundary[ii,0], pos_boundary[ii,0]+vel_boundary[ii,0]*DT],
                      'k--')
            plt.plot([t_offset+time_attr, t_offset+time_attr+DT],
                     [pos_boundary[ii,1], pos_boundary[ii,1]+vel_boundary[ii,1]*DT],
                      'k--')
            
            
        rat = 0.1
        plt.xlim([t_offset-time_attr*rat, t_offset+time_attr*(1+rat)])

        dx_min = 1
        dx = np.max([np.abs(pos_boundary[ii,0] - pos_boundary[ii,1]), dx_min])
        plt.ylim([np.min(pos_boundary[ii,:])-dx*rat,
                  np.max(pos_boundary[ii,:])+dx*rat])

            
    plt.ion()
    plt.show()


    # vel = (pos_next - pos_boundary[:, 0])/dt


def __main__():
    simuCase = 1
    if simuCase==0:
        pos_boundary = np.array([[0, 0]])
        vel_boundary = np.array([[1, 1]])

        time_next_attr = 1
        time = 0

    elif simuCase==1:
        pos_boundary = np.array([[0., 1.37727118]])
        vel_boundary = np.array([[ 0., -0.3188507]])

        time_next_attr = 0.01+0.04
        time = 0.2

    DT = 0.04

    spline_factors = np.zeros((1,4))
    
    N_JOINTS = 1

    spline_factors[:, 0] = pos_boundary[:, 0]
    spline_factors[:, 1] = vel_boundary[:, 0]

    # import pdb; pdb.set_trace()
    # Time to next goal
    dt = np.max([time_next_attr - time, DT*1.0])
    print('dt_spline', dt)
    print('pos_boundary', pos_boundary)
    print('vel_boundary', vel_boundary)

    c_matr = np.array(([[dt**2, dt**3],
                        [2*dt, 3*dt**2]]))
    c_matr_inv = LA.inv(c_matr)

    for ii in range(N_JOINTS):
        c_vect = np.array(([pos_boundary[ii, 1]-spline_factors[ii,0]-spline_factors[ii,1]*dt, vel_boundary[ii, 1]-spline_factors[ii,1]]))

        c_vect = c_matr_inv.dot(c_vect)
        spline_factors[ii,2] = c_vect[0]
        spline_factors[ii,3] = c_vect[1]
    print('matr', c_matr)
    print('inv', c_matr_inv)
    print('vect', c_vect)
    print('spline factors', spline_factors)

    calculate_cuver(DT, spline_factors, pos_boundary, vel_boundary, time_attr=dt, t_offset=time)

__main__()
