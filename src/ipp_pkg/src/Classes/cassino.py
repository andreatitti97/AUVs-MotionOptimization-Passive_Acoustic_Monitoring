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
    def __init__(self,n_auv, bool, phi=0, y=0):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        '''
        self.__x = None
        if bool == True:
            self.__phi = phi
            self.__y = y
        else:
            self.__phi = []
            self.__y = []
        self.__C = matlib.zeros((1,4))
        self.__bool = bool
        self.t_prev = 0
        self.target_pose = []
        
    @property
    def current_estimate(self):

        return (self.__x, self.__phi, self.__y)

    def init_state_vector(self, x,y, vx, vy, curr_time):

        self.t_prev = curr_time        
        self.__x = np.matrix([[x,y,vx,vy]]).T
        self.target_pose = [x,y,vx,vy]

    def propagation(self, prev_time, curr_time):

        dt = curr_time - prev_time #tempo corrente - tempo a cui sono state fatte le misure
        tmp = np.zeros((len(self.__y),1))
        for i in range(len(self.__y)):
            tmp[i] = self.__y[i]
        
        self.__F = np.matrix([[1,0,dt,0],
                              [0,1,0,dt],
                              [0,0,1,0],
                              [0,0,0,1]])

        self.__x = self.__F*self.__x#stato attuale aggiornato con le misure ricevute tra il tempo corrente e il tempo dell'ultimo stato

        self.t_prev = curr_time
        self.target_pose = self.__x
                                
    def iteration(self, t_meas, measures, auv_position_x, auv_position_y, curr_t):

        
        if self.__bool == True:
            self.__y = [self.__y[0], self.__y[1], self.__y[2], self.__y[3]]
        self.__y.append(auv_position_x*np.sin(measures) - auv_position_y*np.cos(measures))
        if len(self.__y) > 4:
            self.__y.pop(0)

        tmp = np.zeros((len(self.__y),1))
        for i in range(len(self.__y)):
            tmp[i] = self.__y[i]

        delta = (t_meas - self.t_prev)
        self.__C = np.array([np.sin(measures), -np.cos(measures), 0, 0]) #TODO con Cassino non stima la velocità :( (t_meas/curr_t)*
        self.__phi.append(self.__C)
        if len(self.__phi)>4:
            self.__phi.pop(0)

        self.__x =  np.dot(np.linalg.pinv(self.__phi),tmp) #lo stato al tempo t_meas

        xt = self.target_pose[0]
        yt = self.target_pose[1]
        vx = (self.__x[0] - xt) / delta
        vy = (self.__x[1] - yt) / delta
        self.__x[2] = vx
        self.__x[3] = vy






    '''
    def propagation(self, prev_time, curr_time):
        dt = curr_time - prev_time
        #print('cur_time;',curr_time)
        #print('prev_time;',prev_time)

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


        if self.__bool == True:
            self.__y = [self.__y[0], self.__y[1], self.__y[2], self.__y[3]]
        self.__y.append(auv_position_x*np.sin(measures) - auv_position_y*np.cos(measures))
        if len(self.__y) > 4:
            self.__y.pop(0)

        tmp = np.zeros((len(self.__y),1))

        for i in range(len(self.__y)):
            tmp[i] = self.__y[i]

        self.__C = [np.sin(measures), -np.cos(measures), (curr_t - t_meas)*np.sin(measures), -(curr_t - t_meas)*np.cos(measures)]
        # Pre compute for the kalman gain K
        
        self.__phi.append(self.__C)
        if len(self.__phi)>4:
            self.__phi.pop(0)

        self.__x = self.__phi*tmp
    '''