#Import basic system modules
import os
import time
import importlib.util
import numpy as np
from numpy import append, matlib
# Import Costum classes
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
tmp = []
def state_vector_to_scalars(state_vector):
    '''
    Returns the elements from the state_vector as a tuple of scalars.
    '''
    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0],state_vector[3][0,0])    
    
class Estimator:
    def __init__(self, bool, phi=0, y=0):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        '''
        self.__x = None
        self.__bool = bool
        if self.__bool == True:
            self.__phi = phi
            self.__y = y
            #self.__phi = []
            #self.__y = []
        else:
            self.__phi = []
            self.__y = []
        self.__C = matlib.zeros((1,4))
        self.__bool = bool
        self.t_prev = 0

    @property
    def current_estimate(self):
        return (self.__x, self.__phi, self.__y)

    def init_state_vector(self):
        return True

    def propagation(self, curr_time, prev_time):#propagation to the actual state

        tmp_y = np.zeros((len(self.__y),1))
        for i in range(len(self.__y)):
            tmp_y[i] = self.__y[i]
        tmp_phi = np.zeros((len(self.__y),4))
        for i in range(len(self.__y)):
            a = self.__phi[i]
            tmp_phi[i,:] = [a[0],a[1],a[2],a[3]]

        '''self.____phi = [[self.__phi[0],self.__phi[1],self.__phi[2],self.__phi[3]],
        [self.__phi[4],self.__phi[5],self.__phi[6],self.__phi[7]],
        [self.__phi[8],self.__phi[9],self.__phi[10],self.__phi[11]],
        [self.__phi[12], self.__phi[13],self.__phi[14],self.__phi[15]]]'''
        #print('PHIIIIIIIIIIIIIII',self.__phi)
        #print('tmp phi',tmp_phi)
        #print('YYYYYYYYYYYYYYYY',self.__y)
        self.__x =  np.dot(np.linalg.pinv(tmp_phi),tmp_y)
        #time.sleep(50)
        dt = curr_time - prev_time #tempo attuale - tempo ultimo stato noto.
        self.__F = np.matrix([[1,0,dt,0],
                              [0,1,0,dt],
                              [0,0,1,0],
                              [0,0,0,1]])
        self.__x = self.__F*self.__x

    def iteration(self, t_meas, measures, auv_position_x, auv_position_y, prev_t):

        self.__y.append(auv_position_x*np.sin(measures) - auv_position_y*np.cos(measures))
        if len(self.__y) > config.TP:
            self.__y.pop(0)

        delta = (t_meas - prev_t)
        #self.__C = np.array([np.sin(measures), -np.cos(measures), delta*np.sin(measures), -delta*np.cos(measures)])
        self.__C = [np.sin(measures), -np.cos(measures), delta*np.sin(measures), -delta*np.cos(measures)]
        self.__phi.append(self.__C)

        if len(self.__phi) > config.TP:
            self.__phi.pop(0)

        

