import numpy as np
import pylab

# TODO, DIRTY HACK ###################################
#import sys
#sys.path.insert(0, "/home/nickolai/OpenINS2/OpenINS/")
######################################################


from orientationmath.orientation import euler2quat

def create_profile(dt, pf, init):
    """
    Creates ideal rotation of 3-axis table
    pf[:,0]: time in sec
    pf[:,1]: wx, angular speed
    pf[:,2]: init angle x, 
    pf[:,3]: wy, angular speed
    pf[:,4]: init angle y, 
    pf[:,5]: wz, angular speed
    pf[:,6]: init angle z, 
    """

    pf[:,2] += init[0]
    pf[:,4] += init[0]
    pf[:,6] += init[0]


    init_quat = euler2quat(init[0],init[1], init[2])
    
    print 'init_quat = ', init_quat

    (row, col) = np.shape(pf)

    time = np.arange(0, np.sum(pf[:,0]),dt)

    #print 'time = ', time
    euler = np.zeros(row)
    w = np.zeros(row)


    for i in range(0, row):

        wx, roll = rotate(dt, pf[i, 0],  pf[i, 1], pf[i, 2])
        wy, pitch = rotate(dt, pf[i, 0],  pf[i, 1+2], pf[i, 2+2])
        wz, yaw = rotate(dt, pf[i, 0],  pf[i, 1+4], pf[i, 2+4])

        if i == 0.:
            euler = np.array([roll, pitch, yaw])
            w = np.array([wx, wy, wz])
        else:
            euler = np.append(euler, np.array([roll, pitch, yaw]),axis=1)
            w = np.append(w, np.array([wx, wy, wz]), axis=1)

    return time, w, euler

def rotate(dt, time, rate, angle):
    """
    Form angular movement of given axis
    """
    angle = np.arange(0, time, dt) * rate + angle
    rate =  np.ones(len(np.arange(0, time, dt))) * rate

    return rate, angle

def plot_profile(time, angle):
    """
    Plot profile versus time.

    Parameters
    ----------
    time: array, seconds
    angle: array, rad
    """
    pylab.plot(time, angle)
    pylab.xlabel('$time (s)$')
    pylab.ylabel('$\sigma(\\tau)$')
    pylab.title('Allan deviation')
    #pylab.legend('a_x','a_y','a_z')
    pylab.grid(True)
    pylab.show()
 

if __name__ == '__main__':

    pf = np.array([[10., 0,      0,     0, 0,  0, 0],
                   [20., np.pi/20., 0,  0, 0,  0, 0],
                   [10., 0,     np.pi,  0, 0,  0, 0]])


    pf = np.array([[10, 0,        0,        0, 0,  0,     0],
                   [20, np.pi/20, 0,        0, 0,  0,     0],
                   [10, 0,        np.pi,    0, 0,  0,     0],
                   [20, np.pi/20, np.pi,    0, 0,  0,     0],
                   [10, 0,        2*np.pi,  0, 0,  0,     0],
                   [20, 0,        2*np.pi,  0, 0,  np.pi/20., 0],
                   [10, 0,        2*np.pi,  0, 0,  0,   np.pi],
                   [30, 0,        2*np.pi,  0, 0,  0,   np.pi]])

    


    dt = 0.1
    base1 = np.array([0, 0, 0])
    base2 = np.array([-np.pi/2., -np.pi/2., 0.])
    base3 = np.array([np.pi/2., 0, np.pi/2.]) 
    
    time1, rate1, angle1 = create_profile(dt,pf,base1)
    time2, rate2, angle2 = create_profile(dt,pf,base2)
    time3, rate3, angle3 = create_profile(dt,pf,base3)

    print 'lent time = ', len(time1)
    print 'len angle = ', np.shape(angle1)
    print 'len wx = ', np.shape(rate1)
    
    plot_profile(time1, angle1.T)
    plot_profile(time2, angle2.T)
    plot_profile(time3, angle3.T)






#function [ time euler_id w_id  init_quat] = create_profile(dt, pf )
#%UNTITLED2 Summary of this function goes here
#%   Detailed explanation goes here
#
#time = 0:dt:(sum(pf(:,1)))-dt;
#[row col] = size(pf);
#
#t = pf(:,1);
#
#init_quat = angle2quat(pf(1,3), pf(1,5), pf(1,7),'XYZ');
#
#
#for i = 1:row
#if i ==1
#seg_a1_1 = [0:dt:t(i)-dt].*pf(i,2);
#seg_a1_2 = [0:dt:t(i)-dt].*pf(i,4);
#seg_a1_3 = [0:dt:t(i)-dt].*pf(i,6);
#
#seg_w1_1 = ones(1,length(seg_a1_1)).*pf(i,2);
#seg_w1_2 = ones(1,length(seg_a1_1)).*pf(i,4);
#seg_w1_3 = ones(1,length(seg_a1_1)).*pf(i,6);
#
#euler_id = [seg_a1_1' seg_a1_2' seg_a1_3'];
#            w_id = [seg_w1_1' seg_w1_2' seg_w1_3'];
#else
#
#seg_a1_1 = [0:dt:t(i)-dt].*pf(i,2) + pf(i,3);
#seg_a1_2 = [0:dt:t(i)-dt].*pf(i,4) + pf(i,5);
#seg_a1_3 = [0:dt:t(i)-dt].*pf(i,6) + pf(i,7);
#
#seg_w1_1 = ones(1,length(seg_a1_1)).*pf(i,2);
#seg_w1_2 = ones(1,length(seg_a1_1)).*pf(i,4);
#seg_w1_3 = ones(1,length(seg_a1_1)).*pf(i,6);
#
#euler_id = [euler_id;
#seg_a1_1' seg_a1_2' seg_a1_3'];
#w_id = [w_id;
#seg_w1_1' seg_w1_2' seg_w1_3'];
#
#end
#end
#
#end
