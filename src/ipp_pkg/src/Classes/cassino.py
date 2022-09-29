import numpy as np
from math import atan2, pi
from numpy import append, matlib
import time
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

        dt = curr_time - prev_time #tempo attuale - tempo ultimo stato noto.
        self.__F = np.matrix([[1,0,dt,0],
                              [0,1,0,dt],
                              [0,0,1,0],
                              [0,0,0,1]])
        self.__x = self.__F*self.__x

    def iteration(self, t_meas, measures, auv_position_x, auv_position_y, prev_t):


        if self.__bool == True:
            self.__y = [self.__y[0], self.__y[1], self.__y[2], self.__y[3]]
        self.__y.append(auv_position_x*np.sin(measures) - auv_position_y*np.cos(measures))
        if self.__bool == True:
            if len(self.__y) > 4:
                self.__y.pop(0)
        else:
            if len(self.__y) > 8:
                self.__y.pop(0)

        tmp = np.zeros((len(self.__y),1))
        for i in range(len(self.__y)):
            tmp[i] = self.__y[i]

        delta = (t_meas - prev_t)
        self.__C = np.array([np.sin(measures), -np.cos(measures), delta*np.sin(measures), delta*np.cos(measures)])
        self.__phi.append(self.__C)
        if self.__bool == True:
            if len(self.__phi) > 4:
                self.__phi.pop(0)
        else:
            if len(self.__phi) > 8:
                self.__phi.pop(0)    

        self.__x =  np.dot(np.linalg.pinv(self.__phi),tmp)

