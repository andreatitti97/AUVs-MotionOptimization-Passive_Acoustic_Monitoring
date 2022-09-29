#!/usr/bin/env python
#Import basic system modules
import os
import time
import importlib.util
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
spec = importlib.util.spec_from_file_location("module.tracker", class_path+"/tracker_cassino.py")
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)
spec = importlib.util.spec_from_file_location("module.controller", class_path+"/controller.py")
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)
spec = importlib.util.spec_from_file_location("module.sensor", class_path+"/sensor.py")
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)
spec = importlib.util.spec_from_file_location("module.robot", class_path+"/robot.py")
robot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(robot)
# PATH DEFINITON
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')
# Time counter
t = 0
# INIT lists for plot
target_x_traj, target_y_traj, platform_x, platform_y = [], [], [], []
est1_x, est1_y, est2_x, est2_y,est3_x, est3_y,est4_x, est4_y = [], [], [], [], [], [], [], []
auv1_x, auv1_y, auv2_x, auv2_y,auv3_x,auv3_y,auv4_x,auv4_y  = [], [], [], [], [], [], [], []
rmse, bearing1, bearing2 = [], [], []
t_axe, est1_vx, est1_vy = [], [], []
comm_counter = [0, 0, 0, 0]

def sensorPlacement(auv):
    
    for i in range(config.N_AUV): #TODO: AUV up to 6 consider
        if config.ALONG_BAR_FORMATION == False:
            if (i+1) % 2 == 0:
                if i+1 > 3:
                    auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS,
                        -1,config.BASELINE_X, config.BASELINE_Y-config.BASELINE_Y/2))#freq,mean,variance,displachement
                else:   
                    auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS,
                        -1,0,config.BASELINE_Y))#freq,mean,variance,displachement
            if (i+1) % 2 == 1:
                if i+1 > 2:
                    auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS,1,
                        config.BASELINE_X, config.BASELINE_Y-config.BASELINE_Y/2))#freq,mean,variance,displachement
                else:   
                    auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS,
                        1,0,config.BASELINE_Y))#freq,mean,variance,displachement
        else:
            if (i+1) % 2 == 0:
                if i+1 > 3:
                    auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS,
                        -1,0, config.BASELINE_Y))#freq,mean,variance,displachement
                else:   
                    auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS,
                        -1,0,config.BASELINE_Y/2))#freq,mean,variance,displachement
            if (i+1) % 2 == 1:
                if i+1 > 2:
                    auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS,1,
                        0, config.BASELINE_Y/2))#freq,mean,variance,displachement
                else:   
                    auv.append(sensor.Sensor(str(i),1,0,config.SIGMA_MEAS,
                        1,0,config.BASELINE_Y))#freq,mean,variance,displachement
    return auv

def estimation_routine(count1,obs,delta_time,meas_table,flag,i,comm_counter,t):

    if count1 % config.MEAS_UPDATE == 0:
        obs[i].processMeasurement(meas_table[0])
    if delta_time[i] > 2 and flag[i] == True:
        comm_counter[i] += 1
        if comm_counter[i] != 7: #90 %
            if config.ALONG_BAR_FORMATION == True:
                idx1 = 1
                idx2 = 2
                idx3 = 3
            else:
                idx1 = 2
                idx2 = 3
                idx3 = 1
            obs[i].processMeasurement(meas_table[idx1])
            obs[i].processMeasurement(meas_table[idx2]) 
        if comm_counter[i] == 4 or comm_counter[i] == 8 or comm_counter[i] == 1: #30 %
            obs[i].processMeasurement(meas_table[idx3])
        delta_time[i] = 0
        flag[i] = False
        if comm_counter[i] > 10:
            comm_counter[i] = 0
    if count1 % config.STATE_PROPAGATION == 0:
        obs[i].propagate_estimation(t)

    return obs, comm_counter, flag

def run_simulation(robots, obs, auv, pub, poly_traj):
    """Simulate the sensor platform and the moving target"""
    global count1, comm_counter

    Hz = 1/(config.TIME_STEP) #NB: different from sampling rate for move things, this is ros rate
    rate = rospy.Rate(Hz)
    # Init Time Variables
    t = 0    
    t0 = 0
    count1 = 0
    cmds = []
    flag = [False, False, False, False]
    delta_time = [0,0,0,0]
    while t <= config.TIME_DURATION:
        rospy.loginfo('SIMULATION TIME(s)')
        rospy.loginfo(t)
        auv_pose = []
        if count1 % config.MEAS_UPDATE == 0 or count1 == 0: #IN QUESTA FASE METTTI SOLO LA MISURA LOCALE
            measures, rel_bearing, t_meas, meas_pose, meas_table  = [], [], [], [], []

        for instance in robots:
        # SIMULATE SENSORS MEASURAMENTS - rimane uguale
            
            for i in range(len(auv)):
                
                auv[i].vehiclePose(instance.pose.x,instance.pose.y,instance.pose.theta)
                auv[i].targetPoseReal(instance.pose_target.x,instance.pose_target.y,instance.pose_target.theta)
                [measure_, auv_pose_, rel_bearing_] = auv[i].measureBearing()
                auv_pose.append(auv_pose_)
                measures.append(measure_)
                t_meas.append(t) 
                meas_pose.append(auv_pose_)
                rel_bearing.append(rel_bearing_)
            if count1 % config.MEAS_UPDATE == 0 or count1 == 0: #IN QUESTA FASE METTTI SOLO LA MISURA LOCALE

                for i in range(len(measures)):#create a matrix with measuraments and timestamp
                    tmp = meas_pose[i]
                    arr = [t_meas[i],measures[i],tmp[0],tmp[1]]
                    meas_table.append(arr)
                    t0 = t
                for i in range(len(auv)):
                    flag[i] =  True
            for i in range(len(auv)):
                delta_time[i] = t - t0
            platform_pose = np.array([instance.pose.x,instance.pose.y,instance.pose.theta])
            
            target_state_real = [instance.pose_target.x,instance.pose_target.y, 
            instance.lin_vel_target*np.cos(instance.pose_target.theta),
            instance.lin_vel_target*np.sin(instance.pose_target.theta)]

            platform_x.append(instance.pose.x)
            platform_y.append(instance.pose.y)
            target_x_traj.append(instance.pose_target.x)
            target_y_traj.append(instance.pose_target.y)

        # SIMULATE the ESTIMATIONS
        for i in range(len(auv)):
            obs, comm_counter, flag=estimation_routine(count1,obs,delta_time,meas_table,flag,i,comm_counter,t)
                
        # SAVE DATA FOR PLOT
        #Estimation Data
        if count1 >= 15:
            curr_est1, phi1, y1 = obs[0].state
            curr_est2, phi2, y2 = obs[1].state
            if config.N_AUV > 2:
                curr_est3, phi3, y3 = obs[2].state
                curr_est4, phi4, y4 = obs[3].state

            t_axe.append(t)
            est1_x.append(curr_est1[0,0])
            est1_y.append(curr_est1[1,0])
            est1_vx.append(curr_est1[2,0])
            est1_vy.append(curr_est1[3,0])
            est2_x.append(curr_est2[0,0])
            est2_y.append(curr_est2[1,0])
            if config.N_AUV > 2:
                est3_x.append(curr_est3[0,0])
                est3_y.append(curr_est3[1,0])
                est4_x.append(curr_est4[0,0])
                est4_y.append(curr_est4[1,0])
            
            err_x = np.sqrt(((target_state_real[0] - curr_est1[0,0])**2))
            err_y = np.sqrt(((target_state_real[1] - curr_est1[1,0])**2))
            norma_err = np.sqrt(err_x**2+err_y**2)
            rmse.append(norma_err)
            for i in range(len(auv_pose)):  
                tmp = auv_pose[i]
                if i == 0:
                    auv1_x.append(tmp[0])
                    auv1_y.append(tmp[1])
                if i == 1:
                    auv2_x.append(tmp[0])
                    auv2_y.append(tmp[1])
                if i == 2:
                    auv3_x.append(tmp[0])
                    auv3_y.append(tmp[1])
                if i == 3:
                    auv4_x.append(tmp[0])
                    auv4_y.append(tmp[1])

        # SEND LAST INFORMATIONS and LOAD SEQUENCE OF CTRL_CMD FROM OPTIMIZATION
        if count1%(config.TIME_COUNTER) == 0 and config.OPTIMIZATION_ON == True and count1 != 0: 
            
            rospy.loginfo('SENDING DATA')
            pub[0].publish(np.array(curr_est1,dtype=np.float32))
            rospy.sleep(config.TIME_STEP*5)
            pub[1].publish(np.array(platform_pose,dtype=np.float32))
            rospy.sleep(config.TIME_STEP*5)
            phi = []
            for i in range(config.N_AUV):        
                tmp = phi1[i]
                for j in range(config.N_AUV):
                    phi.append(tmp[j])
            pub[2].publish(np.array(phi,dtype=np.float32))
            rospy.sleep(config.TIME_STEP*5)
            pub[3].publish(np.array(y1,dtype=np.float32))
            rospy.sleep(config.TIME_STEP*5)
            cmds = rospy.wait_for_message('ctrl_cmd',numpy_msg(Floats))
            cmds = cmds.data
            rospy.loginfo('RECEIVED CMDS')
        
        ######################################### MOVE THE ROBOTS #######################################################
        instance.move(config.TIME_STEP*config.TIME_SCALER, cmds, count1)
        instance.move_target(config.TIME_STEP*config.TIME_SCALER,poly_traj[count1])
        #################################################################################################################
        if int(t) == (config.TIME_DURATION-1):
            rospy.loginfo('saving data for plot')
            
            np.savetxt(plot_path+'/target_x_traj.txt',target_x_traj)
            np.savetxt(plot_path+'/target_y_traj.txt',target_y_traj)

            if config.OPTIMIZATION_ON == True:
                np.savetxt(plot_path+'/est1_x_ON.txt',est1_x)
                np.savetxt(plot_path+'/est1_y_ON.txt',est1_y)
                np.savetxt(plot_path+'/est2_x_ON.txt',est2_x)
                np.savetxt(plot_path+'/est2_y_ON.txt',est2_y)
                if config.N_AUV > 2:
                    np.savetxt(plot_path+'/est3_x_ON.txt',est3_x)
                    np.savetxt(plot_path+'/est3_y_ON.txt',est3_y)
                    np.savetxt(plot_path+'/est4_x_ON.txt',est4_x)
                    np.savetxt(plot_path+'/est4_y_ON.txt',est4_y)
                np.savetxt(plot_path+'/rmse_ON.txt',rmse)
                np.savetxt(plot_path+'/x_platform_ON.txt',platform_x)
                np.savetxt(plot_path+'/y_platform_ON.txt',platform_y)
                np.savetxt(plot_path+'/auv1_x_ON.txt',auv1_x)
                np.savetxt(plot_path+'/auv1_y_ON.txt',auv1_y)
                np.savetxt(plot_path+'/auv2_x_ON.txt',auv2_x)
                np.savetxt(plot_path+'/auv2_y_ON.txt',auv2_y)
                if config.N_AUV > 2:
                    np.savetxt(plot_path+'/auv3_x_ON.txt',auv3_x)
                    np.savetxt(plot_path+'/auv3_y_ON.txt',auv3_y)
                    np.savetxt(plot_path+'/auv4_x_ON.txt',auv4_x)
                    np.savetxt(plot_path+'/auv4_y_ON.txt',auv4_y)
            else:

                np.savetxt(plot_path+'/est1_x_OFF.txt',est1_x)
                np.savetxt(plot_path+'/est1_y_OFF.txt',est1_y)
                np.savetxt(plot_path+'/est2_x_OFF.txt',est2_x)
                np.savetxt(plot_path+'/est2_y_OFF.txt',est2_y)
                if config.N_AUV > 2:
                    np.savetxt(plot_path+'/est3_x_OFF.txt',est3_x)
                    np.savetxt(plot_path+'/est3_y_OFF.txt',est3_y)
                    np.savetxt(plot_path+'/est4_x_OFF.txt',est4_x)
                    np.savetxt(plot_path+'/est4_y_OFF.txt',est4_y)
                np.savetxt(plot_path+'/rmse_OFF.txt',rmse)
                np.savetxt(plot_path+'/x_platform_OFF.txt',platform_x)
                np.savetxt(plot_path+'/y_platform_OFF.txt',platform_y)
                np.savetxt(plot_path+'/auv1_x_OFF.txt',auv1_x)
                np.savetxt(plot_path+'/auv1_y_OFF.txt',auv1_y)
                np.savetxt(plot_path+'/auv2_x_OFF.txt',auv2_x)
                np.savetxt(plot_path+'/auv2_y_OFF.txt',auv2_y)
                if config.N_AUV > 2:
                    np.savetxt(plot_path+'/auv3_x_OFF.txt',auv3_x)
                    np.savetxt(plot_path+'/auv3_y_OFF.txt',auv3_y)
                    np.savetxt(plot_path+'/auv4_x_OFF.txt',auv4_x)
                    np.savetxt(plot_path+'/auv4_y_OFF.txt',auv4_y)
        
        t += config.TIME_STEP*config.TIME_SCALER
        count1 += 1
        rate.sleep()

def main():
    # ROS INIT
    rospy.init_node('simulation')
    pub = []
    pub_estimation = rospy.Publisher('estimation', numpy_msg(Floats), queue_size=10)
    pub_platform_state = rospy.Publisher('platform_state', numpy_msg(Floats), queue_size=100)
    pub_regressor = rospy.Publisher('regressor', numpy_msg(Floats), queue_size=100)
    pub_uscita = rospy.Publisher('output', numpy_msg(Floats), queue_size=100)
    pub.append(pub_estimation)
    pub.append(pub_platform_state)
    pub.append(pub_regressor)
    pub.append(pub_uscita)
    # Initial Conditions
    pose_target = config.Pose(config.TARGET_INIT[0], config.TARGET_INIT[1],  config.TARGET_INIT[2])
    pose_start_1 = config.Pose(config.PLATFORM_INIT_POSE[0], config.PLATFORM_INIT_POSE[1], config.PLATFORM_INIT_POSE[2])
    target_start = np.array([config.TARGET_INIT[0],config.TARGET_INIT[1], config.TARGET_INIT[2]])
    target_goal = np.array([config.TARGET_GOAL[0], config.TARGET_GOAL[1],config.TARGET_GOAL[2]])
    # Init tracker controller and robots
    tr = []
    tracker1 = tracker.Tracker('first_observer', False)
    tracker2 = tracker.Tracker('second_observer', False)
    tracker3 = tracker.Tracker('third_observer', False)
    tracker4 = tracker.Tracker('forth_observer', False)
    tr.append(tracker1)
    tr.append(tracker2)
    tr.append(tracker3)
    tr.append(tracker4)
    controller1_target = controller.Controller(0.01, 0.1) # controller parameters  (rho,alpha -> gain linear and angul vel) DO NOT CHANGE
    controller1_auv = controller.Controller(1, 0.003) #0.01
    robot_1 = robot.Robot("formation_reference", "y", controller1_auv, controller1_target)
    auv = []
    # Sensor Initialization
    auv = sensorPlacement(auv) #trapezoidal formation
    # Set the AUV and the TARGET to the initial conditions
    robot_1.set_start_target_poses(pose_start_1, pose_target)
    # Instantiate the object Robot 
    robots: list[robot.Robot] = [robot_1]
    # Generate Trajectory given the target start pos and waypoints
    ts = np.linspace(0,config.TIME_DURATION+1000,round(config.TIME_DURATION+1000/(config.TIME_SCALER*config.TIME_STEP)))
    [ts, poly_traj, vel, acc] = config.generatePolynomialTrajectory(ts, target_start, 0, 0, target_goal, 0, 0)
    # Run The Simulation
    if config.OPTIMIZATION_ON == True:
        rospy.loginfo('LAUNCH THE OPTIMIZATION')
        time.sleep(3)# wait for optimization to launch
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION ON')
    else:
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION OFF')

    run_simulation(robots, tr, auv, pub, poly_traj)

if __name__ == '__main__':
    main()
