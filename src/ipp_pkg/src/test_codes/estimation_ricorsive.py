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

def state_vector_to_scalars(state_vector):
    '''
    Returns the elements from the state_vector as a tuple of scalars.
    '''
    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0],state_vector[3][0,0])    
    
class Estimator:
    def __init__(self, bool, id, init_state_vector):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        '''
        self.__x = init_state_vector
        print('INIT STATE VECTOR',self.__x)

        self.__P = np.eye(4)*config.SIGMA_MEAS
        self.__phi = []
        self.__y = []
        self.__C = matlib.zeros((1,4))
        self.__t = []
        self.__r = config.SIGMA_MEAS
        self.__beta = 0.99 #Forgetting Factor
        self.t_prev = 0

    @property
    def current_estimate(self):
        return (self.__x, self.__phi, self.__y) #self.__w

    def init_state_vector(self):
        return True

    def propagation(self, curr_time, prev_time):#compute old state in the regressor and propagation to the actual state
        # Propagate the estimation
        dt = curr_time - prev_time #tempo attuale - tempo ultimo stato noto.
        self.__F = np.matrix([[1,0,dt,0],
                              [0,1,0,dt],
                              [0,0,1,0],
                              [0,0,0,1]])
        tmp = np.zeros((4,1))
        for i in range(4):
            tmp[i] = self.__x[i]

        self.__x[0] = self.__x[0]+dt*self.__x[2]
        self.__x[1] = self.__x[1]+dt*self.__x[3]


    def iteration(self, t_meas, measures, auv_position_x, auv_position_y, prev_t):
        
        self.__y.append(auv_position_x*np.sin(measures) - auv_position_y*np.cos(measures))
        self.__t.append(t_meas)
        
        self.__C = np.array([np.sin(measures), -np.cos(measures), (t_meas - prev_t)*np.sin(measures), -(t_meas - prev_t)*np.cos(measures)])
        self.__phi.append(self.__C)
        #self.__C = np.array([np.sin(measures), -np.cos(measures), 0, 0])


        y = auv_position_x*np.sin(measures) - auv_position_y*np.cos(measures)
        if ((self.__r <= 0) or (self.__beta <= 0) or (self.__beta > 1)):
            ok = 0
            K = 0
            self.__r = 1
            self.beta = 1
            self.__P = self.__P
        else:
            ok = 1
            rho = self.__beta*self.__r + np.dot(np.dot(self.__C,self.__P),np.transpose(self.__C))
            tmp = self.__P
            self.__P = (tmp-(np.dot(np.dot(np.dot(tmp,np.transpose(self.__C)),self.__C),tmp))/rho)/self.__beta
            self.__P = (self.__P+np.transpose(self.__P))/2
            K = np.dot(self.__P,np.transpose(self.__C))/self.__r
        if ok==0:
            print('PROBLEMS+++++++++++++++++++++')
            time.sleep(100)
        print('OLD STATE',self.__x)
        self.__x = self.__x + K*(y-np.dot(self.__C,self.__x))
        print('debug',y-np.dot(self.__C,self.__x))
        print('NEW STATE',self.__x)

        #self.__x[0] = self.__x[0]+(t_meas-prev_t)*self.__x[2]
        #self.__x[1] = self.__x[1]+(t_meas-prev_t)*self.__x[3]
        #print('PROPAGATED STATE',self.__x)
        
        for i in range(len(self.__phi)): # UPDATE REGRESSOR COLUMN in chronologically order
            if prev_t!=self.__t[i]:
                tmp = self.__phi[i]
                tmp[2] = ((self.__t[i]-self.__t[0])/(self.__t[i]-prev_t))*tmp[2]
                tmp[3] = ((self.__t[i]-self.__t[0])/(self.__t[i]-prev_t))*tmp[3]
                self.__phi[i] = [tmp[0],tmp[1],tmp[2],tmp[3]]
       


        
        

        

