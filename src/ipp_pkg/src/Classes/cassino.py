import numpy as np
from math import atan2, pi
from numpy import append, matlib
import time
def state_vector_to_scalars(state_vector):
    '''
    Returns the elements from the state_vector as a tuple of scalars.
    '''
    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0],state_vector[3][0,0])    
    
class Estimator:
    def __init__(self,n_auv):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        '''
        self.__xI = matlib.identity(4)
        self.__x = None
        self.__phi = []
        self.__y = []
        self.__C = matlib.zeros((1,4))
    @property
    def current_estimate(self):
        return (self.__x)

    def init_state_vector(self, x,y, vx, vy):
        
        self.__x = np.matrix([[x,y,vx,vy]]).T

    def propagation(self, prev_time, curr_time):
        dt = curr_time - prev_time
        print('cur_time;',curr_time)
        print('prev_time;',prev_time)
        #print(dt)
        #time.sleep(1)
        self.__F = np.matrix([[1,0,dt,0],
                              [0,1,0,dt],
                              [0,0,1,0],
                              [0,0,0,1]])
        tmp = np.zeros((len(self.__y),1))
        for i in range(len(self.__y)):
            tmp[i] = self.__y[i]
        self.__x = self.__F*np.linalg.pinv(self.__phi)*tmp
        #print('POST PROPAGATION',self.__x)
        #time.sleep(5)
                                
    def iteration(self, t_meas, measures, auv_position_x, auv_position_y, curr_t):

        # Return state estimated
        #[xt, yt, dotx, doty] = state_vector_to_scalars(self.__x)
        #print(xt, yt, dotx, doty)
        
        # Compute the output error for both measuraments.
        self.__y.append(auv_position_x*np.sin(measures) - auv_position_y*np.cos(measures))
        if len(self.__y)>4:
            self.__y.pop(0)
        #self.recompute_PHI(auv_position, bool)
        self.__C = [np.sin(measures), -np.cos(measures), (curr_t - t_meas)*np.sin(measures), -(curr_t - t_meas)*np.cos(measures)]
        # Pre compute for the kalman gain K
        self.__phi.append(self.__C)
        if len(self.__phi)>4:
            self.__phi.pop(0)
        tmp = np.zeros((len(self.__y),1))
        for i in range(len(self.__y)):
            tmp[i] = self.__y[i]
        self.__x = self.__phi*tmp
        #print('AFTER ITERATION#########',self.__x)

