#!/usr/bin/env python
#Import basic system modules
import os
import time
import importlib.util
import matplotlib.pyplot as plt
# Import math modules
from math import pi
import numpy as np
from scipy import stats
#Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
#import matplotlib.pyplot as plt
# Import Costum classes
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
spec = importlib.util.spec_from_file_location("module.tracker", class_path+"/tracker.py")
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)
spec = importlib.util.spec_from_file_location("module.controller", class_path+"/controller.py")
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)
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
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')
# Time counter
t = 0
# INIT lists for plot
target_x_traj, target_y_traj, platform_x, platform_y = [], [], [], []
est1_x, est1_y, est2_x, est2_y,est3_x, est3_y,est4_x, est4_y = [], [], [], [], [], [], [], []
est4_vx, est4_vy = [],[]
auv1_x, auv1_y, auv2_x, auv2_y,auv3_x,auv3_y,auv4_x,auv4_y  = [], [], [], [], [], [], [], []
rmse, rmse_, bearing1, bearing2 = [], [], [],[]
cov1, cov2, cov3, cov4 = [],[],[],[]
prova_x,prova_y = [], []
des1_x, des1_y, des2_x, des2_y, des3_x, des3_y, des4_x, des4_y = [], [], [], [], [], [], [], []
def run_simulation(target, obs, auv, pub, cpf_control, formation, init_orientation):
    """Simulate the sensor platform and the moving target"""
    global count1
    propagation = False
    Hz = 1/(config.TIME_STEP) #NB: different from sampling rate for move things, this is ros rate
    rate = rospy.Rate(Hz)
    # Init Time Variables and counters
    t, count1, j = 0, 0, 0
    meas_table = []
    cmds = []
    for i in range(config.M):
        cmds.append(0)
    delay = []
    flags = []
    for i in range(config.N_AUV):
        delay.append(0.0)
        flags.append(0) 

    # Init AUVs position and orientation according to given formation
    positions = np.zeros((len(auv),2))
    leader_pos = np.array([formation[0,0],formation[0,1]])
    
    for i in range(0, len(auv)):

        positions[i,0] = leader_pos[0] + config.a*(formation[i+1,0]*np.cos(config.PLATFORM_INIT_POSE[2])+formation[i+1,1]*np.sin(config.PLATFORM_INIT_POSE[2]))
        positions[i,1] = leader_pos[1] + config.b*(-formation[i+1,0]*np.sin(config.PLATFORM_INIT_POSE[2])+formation[i+1,1]*np.cos(config.PLATFORM_INIT_POSE[2]))

    orientations = np.zeros(len(auv))   
    leader_ori =init_orientation
    for i in range(len(auv)):
        orientations[i] = init_orientation

    ## SIMULATION LOOP ############################################################################################################
    while t <= config.TIME_DURATION:
        rospy.loginfo('SIMULATION TIME(s)')
        rospy.loginfo(t)

        # Simulate Sensor Measuraments
        for j in range(config.N_AUV):
            # If time to transmit
            if count1 % config.MEAS_UPDATE == 0:
                #Make measurements
                [measure_, rel_bearing_, meas_pos] = auv[j].measureBearing(target.pose_target.x,target.pose_target.y,positions[j], orientations[j])
                arr = [t,measure_,meas_pos[0],meas_pos[1]]
                meas_table.append(arr)
                
                if len(meas_table) > config.TP:
                    meas_table.pop(0)
            
            # Compute the delay of the msg of heach auvs
            if flags[j] == 0 and propagation == False:
                delay[j] += config.TIME_STEP*config.TIME_SCALER    

            # If the delay measured is equal to the expected one the msg is arrived to AUV4
            sigma = np.random.uniform(-config.variance[j], config.variance[j])
            if  delay[j] >= config.mean[j] + sigma:
                delay[j] = 0.0
                flags[j] = 1
                if np.sum(delay) == 0.0 and np.sum(flags)==4:
                    propagation = True

        # SIMULATE the ESTIMATIONS
        if propagation == True:

            obs[0].processMeasurement(meas_table)
            obs[0].propagate_estimation(t)
            flags = [0,0,0,0]
            # Retrieve Estimation
            curr_est, phi, y = obs[0].state
            # Compute Covariance of the target state - USING FORGETTING FACTOR and RE-WEIGHTED estimation over the range-ratio
            R = np.zeros((len(y),len(y))) #matrice diagonale perchè errori sulle singole misure indipendenti tra loro
            
            '''
            UNCOMMENT FOR FORGETTING FACTOR AND RE-WEIGHTED ESTIMATION
            beta = np.zeros((len(y),1))
            time_ = np.linspace(0,1,len(y)) #pesi tempo
            beta = np.e**(time_)
            w = np.zeros((len(range_ratio),1)) 
            for i in range(len(range_ratio)):
                w[i] = range_ratio[i]
            gamma = np.e**(w) # pesi distanza    
            '''
            
            for i in range(len(y)): 
                for j in range(len(y)):
                    if i == j:
                        R[i,j] = (config.SIGMA_MEAS**2)#/(gamma[i]*beta[i]) #for adding re-weighted and 
                    else:
                        R[i,j] = 0
            if count1 > 0:
                cov = np.linalg.inv(np.dot(np.dot(np.transpose(phi),np.linalg.inv(R)),phi))
            else: 
                cov = np.zeros((4,4))
            
            tmp_ = np.dot(np.transpose(phi),np.dot(np.linalg.inv(R),y))
            tmp_2 = np.linalg.inv(np.dot(np.dot(np.transpose(phi),np.linalg.inv(R)),phi))
            x = np.dot(tmp_2,tmp_) # stima pesata su distanza + forgetting factor
             
        ############################################# TRIGGER OPTIMIZATION ##############################################
            if config.OPTIMIZATION_ON == True:
                
                rospy.loginfo('SENDING DATA')
                pub[0].publish(np.array(curr_est,dtype=np.float32))
                rospy.sleep(config.TIME_STEP*10)
                tmp = [leader_pos[0],leader_pos[1],leader_ori]
                pub[1].publish(np.array(tmp,dtype=np.float32))
                rospy.sleep(config.TIME_STEP*10)

                tmp = []
                for i in range(4):
                    for j in range(4):
                        tmp.append(cov[i,j])
                pub[2].publish(np.array(tmp,dtype=np.float32))
                rospy.sleep(config.TIME_STEP*10)
                cmds = rospy.wait_for_message('ctrl_cmd',numpy_msg(Floats))
                cmds = cmds.data

                print('RECEIVED CMDS (deg) -------------------------------------------------',cmds*180/pi)
        
        ##################################################################################################################
             
        ######################################### MOVE THE ROBOTS #######################################################
        [leader_pos,leader_ori, positions, orientations, desired_pos] = cpf_control.move_agents(leader_pos, cmds[0],config.TIME_STEP*config.TIME_SCALER, positions, orientations, count1, False)
        target.move_target(config.TIME_STEP*config.TIME_SCALER)
        
        #################################################################################################################
                      
        ##################### SAVE THE POSITIONS OF TEAM REFERENCE/AGENTS/TARGET/ STATE FOR PLOT ########################
        
        des1_x.append(desired_pos[0,0])
        des1_y.append(desired_pos[0,1])
        des2_x.append(desired_pos[1,0])
        des2_y.append(desired_pos[1,1])
        des3_x.append(desired_pos[2,0])
        des3_y.append(desired_pos[2,1])
        des4_x.append(desired_pos[3,0])
        des4_y.append(desired_pos[3,1])

        platform_x.append(leader_pos[0])
        platform_y.append(leader_pos[1])
        auv1_x.append(positions[0,0])
        auv1_y.append(positions[0,1])
        auv2_x.append(positions[1,0])
        auv2_y.append(positions[1,1])

        auv3_x.append(positions[2,0])
        auv3_y.append(positions[2,1])

        auv4_x.append(positions[3,0])
        auv4_y.append(positions[3,1])

        target_x_traj.append(target.pose_target.x)
        target_y_traj.append(target.pose_target.y)
                
        if propagation == True: # you can start saving estimation after the first state propagation
            
            # Saving DATA ABOUT ESTIMATION
            est4_x.append(curr_est[0,0])
            est4_y.append(curr_est[1,0])
            est4_vx.append(curr_est[2,0])
            est4_vy.append(curr_est[3,0])
            
            prova_x.append(x[0])
            prova_y.append(x[1])

            cov1.append(cov[0,0])
            cov2.append(cov[1,1])
            cov3.append(cov[2,2])
            cov4.append(cov[3,3])

            target_state_real = [target.pose_target.x,target.pose_target.y, 
                target.lin_vel_target*np.cos(target.pose_target.theta),
                target.lin_vel_target*np.sin(target.pose_target.theta)]
                    

            err_x = (target_state_real[0] - curr_est[0,0])
            err_y = (target_state_real[1] - curr_est[1,0])
            err_x_ = (target_state_real[0] - x[0])**2
            err_y_ = (target_state_real[1] - x[1])**2

            e = err_x**2+err_y**2
            e_ = err_x_**2+err_y_**2

            rmse.append(e)
            rmse_.append(e_)
            
            propagation = False
        ##################################################################################################################
        

        #  Stop simulation and save data to .txt files ###################################################################
        if int(t) == (config.TIME_DURATION-1):
            rospy.loginfo('saving data for plot')
            
            np.savetxt(plot_path+'/target_x_traj.txt',target_x_traj)
            np.savetxt(plot_path+'/target_y_traj.txt',target_y_traj)

            if config.OPTIMIZATION_ON == True:

                np.savetxt(plot_path+'/est4_x_ON.txt',est4_x)
                np.savetxt(plot_path+'/est4_y_ON.txt',est4_y)
                np.savetxt(plot_path+'/rmse_ON.txt',rmse)
                np.savetxt(plot_path+'/rmse_improved_ON.txt',rmse_)
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
                np.savetxt(plot_path+'/vx_ON.txt',est4_vx)
                np.savetxt(plot_path+'/vy_ON.txt',est4_vy)
                np.savetxt(plot_path+'/des1_x',des1_x)
                np.savetxt(plot_path+'/des1_y',des1_y)
                np.savetxt(plot_path+'/des2_x',des2_x)
                np.savetxt(plot_path+'/des2_y',des2_y)
                np.savetxt(plot_path+'/des3_x',des3_x)
                np.savetxt(plot_path+'/des3_y',des3_y)
                np.savetxt(plot_path+'/des4_x',des4_x)
                np.savetxt(plot_path+'/des4_y',des4_y)
            else:


                np.savetxt(plot_path+'/est4_x_OFF.txt',est4_x)
                np.savetxt(plot_path+'/est4_y_OFF.txt',est4_y)
                np.savetxt(plot_path+'/rmse_OFF.txt',rmse)
                np.savetxt(plot_path+'/rmse_improved_OFF.txt',rmse_)
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
                np.savetxt(plot_path+'/vx_OFF.txt',est4_vx)
                np.savetxt(plot_path+'/vy_OFF.txt',est4_vy)
                np.savetxt(plot_path+'/prova_x.txt',prova_x)
                np.savetxt(plot_path+'/prova_y.txt',prova_y)
                
                
        t += config.TIME_STEP*config.TIME_SCALER
        
        count1 += 1
        rate.sleep()

def main():
    # ROS INIT
    rospy.init_node('simulation')
    pub = []
    pub_estimation = rospy.Publisher('estimation', numpy_msg(Floats), queue_size=10)
    pub_platform_state = rospy.Publisher('platform_state', numpy_msg(Floats), queue_size=100)
    pub_covariance = rospy.Publisher('covariance', numpy_msg(Floats), queue_size=100)
    pub.append(pub_estimation)
    pub.append(pub_platform_state)
    pub.append(pub_covariance)
    # Initial Conditions
    pose_target = config.Pose(config.TARGET_INIT[0], config.TARGET_INIT[1],  config.TARGET_INIT[2])
    # Set the AUV and the TARGET to the initial conditions
    target_ = target.Target()
    target_.set_start_target_poses(pose_target)      
    
    # Sensor and AUVs Initialization
    auv = []
    for i in range(config.N_AUV): #TODO: AUV up to 6 consider
        auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS))

    # Init trackers 
    trackers = []
    tracker4 = tracker.Tracker('forth_observer', False)
    trackers.append(tracker4)
        
    # Cooperative Path Following initialization
    cpf_control = cpf.CooperativePathFollowing(config.formation, config.N_AUV, config.PLATFORM_INIT_POSE[2], config.K_att, config.K_rep, config.d_rep, config.AUV_VEL, True)
    
    # Run The Simulation
    if config.OPTIMIZATION_ON == True:
        rospy.loginfo('LAUNCH THE OPTIMIZATION')
        time.sleep(3)# wait for optimization to launch
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION ON')
    else:
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION OFF')
    
    run_simulation(target_, trackers, auv, pub, cpf_control, config.formation, config.PLATFORM_INIT_POSE[2])

if __name__ == '__main__':
    main()
