#!/usr/bin/env python
#Import basic system modules
import os
import time
import importlib.util
import matplotlib.pyplot as plt
# Import math modules
from math import pi, atan2
import numpy as np
from scipy import stats
#Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
#import matplotlib.pyplot as plt
# Import Costum classes
class_path = os.path.abspath('/home/andrea/Desktop/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
spec = importlib.util.spec_from_file_location("module.utils", class_path+"/utils.py")
utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utils)
spec = importlib.util.spec_from_file_location("module.tracker", class_path+"/tracker.py")
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)
spec = importlib.util.spec_from_file_location("module.sensor", class_path+"/sensor.py")
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)
spec = importlib.util.spec_from_file_location("module.target", class_path+"/target.py")
target = importlib.util.module_from_spec(spec)
spec.loader.exec_module(target)
spec = importlib.util.spec_from_file_location("module.cpf", class_path+"/cpf.py")
cpf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cpf)
# PATH DEFINITON
plot_path = os.path.abspath('/home/andrea/Desktop/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

# Init lists for plot
# Target and AUVs
target_x_traj, target_y_traj, platform_x, platform_y = [], [], [], []
auv1_x, auv1_y, auv2_x, auv2_y,auv3_x,auv3_y,auv4_x,auv4_y  = [], [], [], [], [], [], [], []
# Estimation Data
est1_x, est1_y, est2_x, est2_y,est3_x, est3_y,est_x, est_y, est_vx, est_vy = [],[], [], [], [], [], [], [], [], []
cov1, cov2, cov3, cov4, err_quad, cond_phi = [],[],[],[],[],[]

def compute_cost(phi,len_y):
    tmp_phi = np.zeros((len_y,4))
    for i in range(len_y):
        row = phi[i]
        tmp_phi[i,:] = [row[0],row[1],row[2],row[3]]
    PHI = np.dot(np.transpose(tmp_phi[:,0:2]),tmp_phi[:,0:2])
    cost = np.linalg.norm(np.linalg.inv(PHI),ord=2)*np.linalg.norm(PHI,ord=2)
    return cost

def initialize_auvs(geometry,s_pose,f,N_AUV,d):
    auvs_xy = np.zeros((N_AUV,2))
    auvs_theta = np.zeros(N_AUV)   
    for i in range(N_AUV):
        auvs_theta[i] = s_pose[2]
    if geometry == 'line' or geometry == 'line2'or geometry=='polygon':
        s_pose = [s_pose[0],s_pose[1],s_pose[2]]
        for i in range(0, N_AUV):
            auvs_xy[i,0] = s_pose[0] + (f[i,0]*np.cos(s_pose[2])+f[i,1]*np.sin(s_pose[2]))
            auvs_xy[i,1] = s_pose[1] - (-f[i,0]*np.sin(s_pose[2])+f[i,1]*np.cos(s_pose[2]))      
    elif geometry == 'column' or geometry == 'column2':      
        s_pose = [d,s_pose[1],s_pose[2]]
        for i in range(N_AUV):
            auvs_xy[i,0] = s_pose[0]-f[i]
            auvs_xy[i,1] = 0
    return auvs_xy, auvs_theta

def sig(x):
    alpha = -0.003
    gamma = config.d
    return 1/(1+np.e**(alpha*(gamma-x)))

def run_simulation(target, obs, auv, pub, cpf_control, f, s_pose):
    """Simulate the sensor platform and the moving target
    Input:  target : target initial state
            obs : list containing already initialized classes Tracker() (reproduce the local estimations)
            auv : list containing sensors state and methods for measurements
            pub : list containing the publishers
            cpf_control : already initialized class for CPF
            f : choosen geometry
            s_pose : initial s state
    """
    global count1
    propagation = False
    updated = False

    Hz = 1/(config.TIME_STEP) #NB: different from sampling rate for move things, this is ros rate
    rate = rospy.Rate(Hz)
    # Init time variables and counters and lists
    t, count1, j, k, last_cmd, idx_motion = 0, 0, 0, 0, 0, 0
    meas_table, measurements, cmds, delay, flags = [], [], [], [], []
    dt = config.TIME_STEP*config.TIME_SCALER
    N = len(auv)

    geometry = config.geometry
    for i in range(config.M):
        cmds.append(0)
    # Initialize communication parameters
    mean_c = config.mean
    sigma_c = config.variance
    for i in range(N):
        delay.append(0.0)
        flags.append(0) 

    # Initialize Path
    path, path_idx, d, ax, ay = cpf_control.initialize_path(f)
    [rx, ry, ryaw, rk, s] = utils.calc_spline_course(path,dt)
    s_pose[0] = d
    # Init AUVs position and orientation according to given formation
    auvs_xy, auvs_theta = initialize_auvs(geometry,s_pose,f,N,d)
    
    plt.plot(ax, ay, "xb", label="Data points")
    plt.plot(s_pose[0],s_pose[1],'og',label='leader position')
    #print(auvs_xy)
    for i in range(config.N_AUV):
        
        plt.plot(auvs_xy[i,0],auvs_xy[i,1],'ok',label="AUV"+str(i))
    plt.plot(rx, ry, "-r", label="Cubic spline path")
    plt.legend()
    plt.axis('equal')
    plt.show() # uncomment for debugging'''


    v_n = config.AUV_VEL
    ## SIMULATION LOOP ############################################################################################################
    while t <= config.TIME_DURATION:
        rospy.loginfo('SIMULATION TIME(s)')
        rospy.loginfo(t)
        # Simulate Sensor Measuraments
        if N > 1:
            if count1 % config.Tm == 0 and count1 > 0:#periodic measurements update
                for i in range(N):
                # Make measurements
                    [measure_, rel_bearing_, meas_pos] = auv[i].measureBearing(target.pose.x,target.pose.y,auvs_xy[i], auvs_theta[i])
                    arr = [t,measure_,meas_pos[0],meas_pos[1]]
                    # Compute probability of loosing a packet
                    if i == 0:
                        meas_table.append(arr)
                    else:
                        ps_1 = auvs_xy[0]
                        dist = np.sqrt((meas_pos[0]-ps_1[0])**2+(meas_pos[1]-ps_1[1])**2)
                        prob = sig(dist)
                        if (np.random.random() <= prob):
                            #msg received

                            meas_table.append(arr)
                        else:
                            print('LOST PACKETT')
        for i in range(N-1):
            if flags[i] == 0 and propagation == False:
                delay[i] += dt

        for j in range(N):
            # If the delay measured is equal to the expected one the msg is arrived to AUV4
            sigma = np.random.uniform(-sigma_c[j], sigma_c[j])
            if  delay[j] >= mean_c[j] + sigma:
                delay[j] = 0.0
                flags[j] = 1
                if np.sum(delay) == 0.0 and np.sum(flags)==(N) and len(meas_table)>=config.Tf:
                    propagation = True    

        # Process all the measurements and propagate the estimation
        if propagation == True:
            for i in range(N):
                obs[i].processMeasurement(meas_table)
                obs[i].propagate_estimation(t)
            meas_table = []
            flags = [0,0,0,0]
            # Retrieve Estimations (if packet loss the first to have regressor < threh speaks)
            curr_est, phi, y = obs[0].state
            curr_est2, phi2, y2 = obs[1].state
            curr_est3, phi3, y3 = obs[2].state
            curr_est4, phi4, y4 = obs[3].state
            cost = compute_cost(phi,len(y))
            cond_phi.append(cost)
            # Compute Covariance of the target state
            R = np.zeros((len(y),len(y))) #matrice diagonale perchè errori sulle singole misure indipendenti tra loro            
            for i in range(len(y)): 
                for j in range(len(y)):
                    if i == j:
                        R[i,j] = (config.SIGMA_MEAS)
                    else:
                        R[i,j] = 0 
            if count1 > 0 and len(phi)>=4:
                a = config.SIGMA_MEAS
                cov = np.linalg.inv(np.dot(np.dot(np.transpose(phi),np.linalg.inv(a*np.identity(len(y)))),phi))
            else: 
                cov = np.zeros((4,4))
            est_x.append(curr_est[0,0])
            est_y.append(curr_est[1,0])
            est_vx.append(curr_est[2,0])
            est_vy.append(curr_est[3,0])
            # Save Covariance associated 
            cov1.append(cov[0,0])
            cov2.append(cov[1,1])
            cov3.append(cov[2,2])
            cov4.append(cov[3,3])
            # Computte the tracking error
            err_x = (target.pose.x - curr_est[0,0])
            err_y = (target.pose.y - curr_est[1,0])
            e = np.sqrt(err_x**2+err_y**2)
            err_quad.append(e)
            # Reset the flag for propagation
            propagation = False
        ############################################# TRIGGER OPTIMIZATION ##############################################
            if config.OPTIMIZATION_ON == True:

                predicted_pose = np.array(np.zeros(2))
                predicted_pose[0] = curr_est[0,0] + config.OPTIMIZATION_TIME_STEP*curr_est[2]
                predicted_pose[1] = curr_est[1,0] + config.OPTIMIZATION_TIME_STEP*curr_est[3]

                tmp_x = s_pose[0]+config.OPTIMIZATION_TIME_STEP*v_n*np.cos(atan2(s_pose[1],s_pose[0]))
                tmp_y = s_pose[1]+config.OPTIMIZATION_TIME_STEP*v_n*np.sin(atan2(s_pose[1],s_pose[0]))

                p_eucl_dist = np.sqrt((predicted_pose[0]-tmp_x)**2+(predicted_pose[1]-tmp_y)**2)
                eucl_dist = np.sqrt((curr_est[0,0]-s_pose[0])**2+(curr_est[1,0]-s_pose[1])**2)
                
                epsi = eucl_dist*80/100 #for OPT_TIME_STEP c.a. 15 sec

                #print('dist euclidea',eucl_dist)
                #print('predicted_eucl_distance',p_eucl_dist)
                #print('epsi',epsi)
                if eucl_dist > 20:
                    v_n = (eucl_dist-epsi)/config.OPTIMIZATION_TIME_STEP
                else:
                #    print('REACHED THE TARGET')
                    v_n = np.sqrt(curr_est[2]**2+curr_est[3]**2)
                    #s_pose = [curr_est[0,0],curr_est[1,0],curr_est[2,0],curr_est[3,0]]

                #print('desired vel',v_n)
                if v_n >= 4:
                    v_n = 4
                elif v_n <= -4:
                    v_n = -4
                elif 0 <= v_n < 2:
                    v_n = 2
                
                print('nominal vel',v_n)
                #v_n = 1
                cpf_control.v_n = v_n
                
                rospy.loginfo('SENDING DATA')
                pub[0].publish(np.array(curr_est,dtype=np.float32))
                rospy.sleep(10/Hz)
                
                tmp = [s_pose[0],s_pose[1],s_pose[2],v_n]
                print(tmp)
                pub[1].publish(np.array(tmp,dtype=np.float32))
                rospy.sleep(10/Hz)
                ax = cpf_control.ax
                ay = cpf_control.ay
                print('LENGTH AX AY',len(ax))
                pub[2].publish(np.array(ax,dtype=np.float32))
                rospy.sleep(10/Hz)
                pub[3].publish(np.array(ay,dtype=np.float32))
                rospy.sleep(10/Hz)
                tmp = []
                for i in range(4):
                    for j in range(4):
                        tmp.append(cov[i,j])
                pub[4].publish(np.array(tmp,dtype=np.float32))
                rospy.sleep(10/Hz)
                tmp = []
                for i in range(4):
                    for j in range(4):
                        tmp.append(cov[i,j])
                pub[2].publish(np.array(tmp,dtype=np.float32))
                rospy.sleep(10/Hz)
                
                cmds = rospy.wait_for_message('ctrl_cmd',numpy_msg(Floats))
                cmds = cmds.data
                print('RECEIVED CMDS (deg) -------------------------------------------------',cmds*180/pi)
                
                print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++UPDATING PATH')
                int_list = [int(item) for item in rx]
                
                print('len rx',len(rx))
                print(int_list)
                print(idx_motion)
                print(idx_motion+path_idx)
                tmp = rx[idx_motion+path_idx]
                path, d, ax, ay, last_cmd = cpf_control.update_path([cmds[0]],last_cmd,config.OPTIMIZATION_TIME_STEP,ax,ay,d,s_pose)
                [rx, ry, ryaw, rk, s] = utils.calc_spline_course(path,dt)
                
                if config.geometry=='column' or config.geometry=='column2':
                    
                    int_list = [int(item) for item in rx]

                    path_idx = int_list.index(int(tmp),0,-1)
                    idx_motion = 0 

                elif config.geometry=='line' or config.geometry=='line2' or config.geometry=='polygon':    
                    idx_motion = 0 #(you delete from the idx the initial path portion deleted)           

                plt.plot(ax, ay, "xb", label="Data points")
                plt.plot(s_pose[0],s_pose[1],'og',label='leader position')
                #print(auvs_xy)
                for i in range(config.N_AUV):
                    
                    plt.plot(auvs_xy[i,0],auvs_xy[i,1],'ok',label="AUV"+str(i))
                plt.plot(rx, ry, "-r", label="Cubic spline path")
                plt.legend()
                plt.axis('equal')
                plt.show() # uncomment for debugging'''

        #################################################################################################################
        ######################################### MOVE THE ROBOTS #######################################################
        if (idx_motion+path_idx) >= (len(rx)-1):#check if the path is finished, in case update with a straight line
            
            print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++UPDATING PATH')
            #TODO: BUG HERE TO SOLVE
            tmp = rx[idx_motion+path_idx]
            path, d, ax, ay, last_cmd = cpf_control.update_path([0],last_cmd,config.OPTIMIZATION_TIME_STEP,ax,ay,d,s_pose)#1 #go straight, path idx increase of 2 or 3
            [rx, ry, ryaw, rk, s] = utils.calc_spline_course(path,dt)

            if config.geometry=='column' or config.geometry=='column2':

                int_list = [int(item) for item in rx]
                path_idx = int_list.index(int(tmp),0,-1)
                idx_motion = 0

            elif config.geometry=='line' or config.geometry=='line2' or config.geometry=='polygon':
                idx_motion = 0 #(you delete from the idx the initial path portion deleted)
                

            plt.plot(ax, ay, "xb", label="Data points")
            plt.plot(s_pose[0],s_pose[1],'og',label='leader position')
            #print(auvs_xy)
            for i in range(config.N_AUV):
                
                plt.plot(auvs_xy[i,0],auvs_xy[i,1],'ok',label="AUV"+str(i))
            plt.plot(rx, ry, "-r", label="Cubic spline path")
            plt.legend()
            plt.axis('equal')
            plt.show() # uncomment for debugging'''

        else:
            d += cpf_control.v_n*dt #distance travelled on the path
            idx_motion += 1
            
        [rx, ry, ryaw, rk, s] = utils.calc_spline_course(path,dt) # compute reference to follow
        [s_pose, auvs_xy, auvs_theta] = cpf_control.move_agents(path, d,s_pose,dt, auvs_xy, auvs_theta, ryaw[path_idx+idx_motion],rx[path_idx+idx_motion],ry[path_idx+idx_motion],False)
        target.move_target(dt)

        #################################################################################################################
        ##################### SAVE THE POSITIONS OF TEAM REFERENCE/AGENTS/TARGET/ STATE FOR PLOT ########################
        platform_x.append(s_pose[0])
        platform_y.append(s_pose[1])
        auv1_x.append(auvs_xy[0,0])
        auv1_y.append(auvs_xy[0,1])
        if len(auv)>1:
            auv2_x.append(auvs_xy[1,0])
            auv2_y.append(auvs_xy[1,1])
        if len(auv)>2:
            auv3_x.append(auvs_xy[2,0])
            auv3_y.append(auvs_xy[2,1])
            auv4_x.append(auvs_xy[3,0])
            auv4_y.append(auvs_xy[3,1])
        target_x_traj.append(target.pose.x)
        target_y_traj.append(target.pose.y)         
        ##################################################################################################################
        #  Stop simulation and save data to .txt files ###################################################################
        if int(t) == (config.TIME_DURATION-1):
            rospy.loginfo('saving data for plot')
            np.savetxt(plot_path+'/target_x_traj.txt',target_x_traj)
            np.savetxt(plot_path+'/target_y_traj.txt',target_y_traj)
            if config.OPTIMIZATION_ON == True:
                np.savetxt(plot_path+'/est4_x_ON.txt',est_x)
                np.savetxt(plot_path+'/est4_y_ON.txt',est_y)
                np.savetxt(plot_path+'/err_quad_ON.txt',err_quad)
                np.savetxt(plot_path+'/x_platform_ON.txt',platform_x)
                np.savetxt(plot_path+'/y_platform_ON.txt',platform_y)
                np.savetxt(plot_path+'/auv1_x_ON.txt',auv1_x)
                np.savetxt(plot_path+'/auv1_y_ON.txt',auv1_y)
                np.savetxt(plot_path+'/auv2_x_ON.txt',auv2_x)
                np.savetxt(plot_path+'/auv2_y_ON.txt',auv2_y)
                np.savetxt(plot_path+'/auv3_x_ON.txt',auv3_x)
                np.savetxt(plot_path+'/auv3_y_ON.txt',auv3_y)
                np.savetxt(plot_path+'/auv4_x_ON.txt',auv4_x)
                np.savetxt(plot_path+'/auv4_y_ON.txt',auv4_y)
                np.savetxt(plot_path+'/cov1_ON.txt',cov1)
                np.savetxt(plot_path+'/cov2_ON.txt',cov2)
                np.savetxt(plot_path+'/cov3_ON.txt',cov3)
                np.savetxt(plot_path+'/cov4_ON.txt',cov4)
                np.savetxt(plot_path+'/vx_ON.txt',est_vx)
                np.savetxt(plot_path+'/vy_ON.txt',est_vy)
                np.savetxt(plot_path+'/cond_ON',cond_phi)

            else:
                np.savetxt(plot_path+'/est4_x_OFF.txt',est_x)
                np.savetxt(plot_path+'/est4_y_OFF.txt',est_y)
                np.savetxt(plot_path+'/err_quad_OFF.txt',err_quad)
                np.savetxt(plot_path+'/x_platform_OFF.txt',platform_x)
                np.savetxt(plot_path+'/y_platform_OFF.txt',platform_y)
                np.savetxt(plot_path+'/auv1_x_OFF.txt',auv1_x)
                np.savetxt(plot_path+'/auv1_y_OFF.txt',auv1_y)
                np.savetxt(plot_path+'/auv2_x_OFF.txt',auv2_x)
                np.savetxt(plot_path+'/auv2_y_OFF.txt',auv2_y)
                np.savetxt(plot_path+'/auv3_x_OFF.txt',auv3_x)
                np.savetxt(plot_path+'/auv3_y_OFF.txt',auv3_y)
                np.savetxt(plot_path+'/auv4_x_OFF.txt',auv4_x)
                np.savetxt(plot_path+'/auv4_y_OFF.txt',auv4_y)
                np.savetxt(plot_path+'/cov1_OFF.txt',cov1)
                np.savetxt(plot_path+'/cov2_OFF.txt',cov2)
                np.savetxt(plot_path+'/cov3_OFF.txt',cov3)
                np.savetxt(plot_path+'/cov4_OFF.txt',cov4)
                np.savetxt(plot_path+'/vx_OFF.txt',est_vx)
                np.savetxt(plot_path+'/vy_OFF.txt',est_vy)
                np.savetxt(plot_path+'/cond_OFF',cond_phi)
        t += dt
        count1 += 1
        
        
        rate.sleep()

def main():
    # ROS INIT
    rospy.init_node('simulation')
    pub = []
    pub_estimation = rospy.Publisher('estimation', numpy_msg(Floats), queue_size=10)
    pub_platform_state = rospy.Publisher('platform_state', numpy_msg(Floats), queue_size=100)
    pub_waypoints_x = rospy.Publisher('ax', numpy_msg(Floats), queue_size=100)
    pub_waypoints_y = rospy.Publisher('ay', numpy_msg(Floats), queue_size=100)
    pub_cov = rospy.Publisher('cov', numpy_msg(Floats), queue_size=100)
    pub.append(pub_estimation)
    pub.append(pub_platform_state)
    pub.append(pub_waypoints_x)
    pub.append(pub_waypoints_y)
    pub.append(pub_cov)
    # Initial Conditions
    pose = utils.Pose(config.TARGET_INIT[0], config.TARGET_INIT[1],  config.TARGET_INIT[2])
    s_pose = [config.PLATFORM_INIT_POSE[0],config.PLATFORM_INIT_POSE[1],config.PLATFORM_INIT_POSE[2]]
    # Set the AUV and the TARGET to the initial conditions
    target_ = target.Target()
    target_.set_start_target_poses(pose)      

    # Sensor and AUVs Initialization
    auv = []
    N = config.N_AUV
    for i in range(N):
        auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS))

    # Init trackers 
    trackers = []
    tracker1 = tracker.Tracker()
    tracker2 = tracker.Tracker()
    tracker3 = tracker.Tracker()
    tracker4 = tracker.Tracker()

    trackers.append(tracker1)
    trackers.append(tracker2)
    trackers.append(tracker3)
    trackers.append(tracker4)
        
    # Cooperative Path Following initialization
    cpf_control = cpf.CooperativePathFollowing(N, config.K_att, config.K_rep, config.d_rep, False)
    
    # Run The Simulation
    if config.OPTIMIZATION_ON == True:
        rospy.loginfo('LAUNCH THE OPTIMIZATION')
        time.sleep(3)# wait for optimization to launch
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION ON')
    else:
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION OFF')
    
    run_simulation(target_, trackers, auv, pub, cpf_control, config.formation, s_pose)

if __name__ == '__main__':
    main()



'''plt.plot(ax, ay, "xb", label="Data points")
                    plt.plot(s_pose[0],s_pose[1],'og',label='leader position')
                    #print(auvs_xy)
                    for i in range(config.N_AUV):
                        
                        plt.plot(auvs_xy[i,0],auvs_xy[i,1],'ok',label="AUV"+str(i))
                    plt.plot(rx, ry, "-r", label="Cubic spline path")
                    plt.legend()
                    plt.axis('equal')
                    plt.show() # uncomment for debugging'''

                
            
            
            #print('++++++++++++++++++++++++++UPDATING PATH+++++++++++++++++++++++++++++++')
