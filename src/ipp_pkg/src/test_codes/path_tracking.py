import math
import numpy as np
import bisect
import os
import importlib.util
import time
from scipy import interpolate
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.planner", class_path+"/spline_planner.py")
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)
cubicSpline = planner

def calc_spline_course(sp,ds):
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s

def main_2d():  # pragma: no cover
    print("CubicSpline1D 2D test")
    import matplotlib.pyplot as plt
    init_theta = 0 #initial ori
    init_pose = [0,0]
    v_n = 1 #m/s
    d_time = 500 #s

    ax = [0,30,60,90]
    ay = [0,0,0,0]



    formation = [0,0,0,0]
    delta_theta = [-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5,-20,-15,-10,-5] #  X  along path
    
    

    init_pose = [ax[-1],ay[-1]]
    #ax, ay = [], []
    for i in range(len(delta_theta)):
        theta_goal = init_theta+delta_theta[i]
        tmp_x = np.cos(theta_goal*np.pi/180)*v_n*d_time+init_pose[0]#ax[3]
        tmp_y = np.sin(theta_goal*np.pi/180)*v_n*d_time+init_pose[1]#ay[3]
        ax.append(tmp_x)
        ay.append(tmp_y)
        init_theta = theta_goal
        init_pose = [tmp_x, tmp_y]


    ds = 0.1 # [m] distance of each interpolated points ----- > più è altro più il path è segmentato ? 

    sp = cubicSpline.CubicSpline2D(ax, ay)
    print('LUNGHEZZA CURVA1',sp.s[-1])

    '''for i in range(4):
        ax.pop(0)
        ay.pop(0)'''

    ryaw_ = []
    rk_ = []
    rx, ry, ryaw, rk, s = calc_spline_course(sp,ds)


    
    x_p, y_p, yaw = [], [], []
    inital_state = [0, 0, 0]
    v = 1 #m/s
    dt = ds
    for i in range(len(ryaw)):
        ang_vel = (ryaw[i]-inital_state[2])
        inital_state[2] = inital_state[2] + ang_vel*dt
        inital_state[0] = inital_state[0] + v*np.cos(inital_state[2])*dt
        inital_state[1] = inital_state[1] + v*np.sin(inital_state[2])*dt
        
        x_p.append(inital_state[0])
        y_p.append(inital_state[1])

    for i in range(1):
        leader_distance_performed = d_time+i*d_time*5
        x_leader, y_leader = sp.calc_position(leader_distance_performed)#computeDesPose(0,0+i*30,ax,ay)
        print('LEADER_x',x_leader)
        print('LEADER_Y',y_leader)
        
        auv_des_pose_x,auv_des_pose_y = [], []
        
        for i in range(len(formation)):
            '''if i == 3:
                init_pose = [ax[-1],ay[-1]]
                #time.sleep(5)
                for i in range(len(delta_theta)):
                    theta_goal = init_theta+delta_theta[i]
                    tmp_x = np.cos(theta_goal*np.pi/180)*v_n*d_time+init_pose[0]#ax[3]
                    tmp_y = np.sin(theta_goal*np.pi/180)*v_n*d_time+init_pose[1]#ay[3]
                    ax.append(tmp_x)
                    ay.append(tmp_y)
                    init_theta = theta_goal
                    init_pose = [tmp_x, tmp_y]
                sp = CubicSpline2D(ax, ay)'''

            x,y = sp.calc_position(formation[i]+leader_distance_performed)#SE ALL'INIZIO GL IAUV POTREBBERO RISULTARE FUORI DAL PATH
            print('distance',formation[i]+leader_distance_performed)# STANDO DIETRO, quindi andrebbe inizializzata a curva dietro al leader
            #y = interpolate.splrep(ax, ay, formation[i])
            auv_des_pose_y.append(y)
            auv_des_pose_x.append(x)

        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="Data points")
        #plt.plot(x_leader, y_leader, "xk", label="INTERPOLATED POINT"+str(i))
        #for i in range(len(formation)):
        #    plt.plot(auv_des_pose_x[i], auv_des_pose_y[i], "xg", label="INTERPOLATED POINT"+str(i))

        
        plt.plot(rx, ry, "-r", label="Cubic spline path")
        plt.plot(x_p,y_p,"m--")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        auv_des_pose_x, auv_des_pose_y = [], []
    print(len(rx))
    print('LUNGHEZZA CURVA1',sp.s[-1])
    plt.show()



if __name__ == '__main__':
    # main_1d()
    main_2d()
