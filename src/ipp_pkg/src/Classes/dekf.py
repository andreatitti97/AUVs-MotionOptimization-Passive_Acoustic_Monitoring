import numpy as np
from math import atan2, pi
from numpy import matlib

def state_vector_to_scalars(state_vector):
    '''
    Returns the elements from the state_vector as a tuple of scalars.
    '''
    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0],state_vector[3][0,0])    
    
class ExtendedKalmanFilter:
    def __init__(self,n_auv,bool,init_cov=[]):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        '''
        self.__xI = matlib.identity(4)
        self.__x = None
        self.__F = None
        self.__Q = None
        self.n_auv = n_auv
        if bool == True:
            self.__P = init_cov
        else:
            self.__P = np.matrix([[1000,0,0,0], # This are set according to the initial uncertainty choosen
                              [0,1000,0,0],     # if change init uncertainty change these
                              [0,0,100,0],    
                              [0,0,0,100]])   

        self.__H = matlib.zeros((1,4))
        self.__R = np.matrix([[0.01]])
        if self.n_auv == 2:
            self.__H = matlib.zeros((2,4))
            self.__R = np.matrix([[0.01,0],[0,0.01]])
        else:
            self.__H = matlib.zeros((4,4))
            self.__R = np.matrix([[0.01,0,0,0],[0,0.01,0,0],[0,0,0.01,0],[0,0,0,0.01]])

        self.__noise_ax = 0.001 
        self.__noise_ay = 0.001
        self.trackingDataState = []

    @property
    def current_estimate(self):
        return (self.__x, self.__P)

    def init_state_vector(self, x,y, vx, vy):
        
        self.__x = np.matrix([[x,y,vx,vy]]).T

    def recompute_F_and_Q(self, dt):
        '''
        updates the motion model and process covar based on delta time from last measurement.
        '''
   
        #set F [4x4]
        self.__F = np.matrix([[1,0,dt,0],
                              [0,1,0,dt],
                              [0,0,1,0],
                              [0,0,0,1]])
        
        #set Q
        dt2 = dt**2
        dt3 = dt**3
        dt4 = dt**4

        e11 = dt4 * self.__noise_ax / 4
        e13 = dt3 * self.__noise_ax / 2
        e22 = dt4 * self.__noise_ay / 4
        e24 = dt3 * self.__noise_ay / 2
        e31 = dt3 * self.__noise_ax / 2
        e33 = dt2 * self.__noise_ax
        e42 = dt3 * self.__noise_ay / 2
        e44 = dt2 * self.__noise_ay

        self.__Q = np.matrix([[e11, 0, e13, 0], 
                              [0, e22, 0, e24],
                              [e31, 0, e33, 0],
                              [0, e42, 0, e44]]) #Q matrix represents accelerations that allows 
                              #the tracked object to deviate from constant velocity.
                              
    def recompute_H(self, auv_positions, bool):

        px,py, vx, vy = state_vector_to_scalars(self.__x)
        #calculate_jacobian of the current state.
        rx = []
        ry = []
 
        if bool == False:#TODO migliora sta condizione
            rx = px - auv_positions[0]
            ry = py - auv_positions[1]
            self.__H = np.matrix([[-ry/(ry**2+rx**2), rx/(rx**2+ry**2) , 0, 0]])
            self.__R = np.matrix([[0.01]])
        else:
            
            for i in range(len(auv_positions)):
                tmp = auv_positions[i]
                rx.append(px - tmp[0])
                ry.append(py - tmp[1])
            self.__H = np.matrix([[-ry[0]/(ry[0]**2+rx[0]**2), rx[0]/(rx[0]**2+ry[0]**2) , 0, 0],
                                    [-ry[1]/(ry[1]**2+rx[1]**2), rx[1]/(rx[1]**2+ry[1]**2) , 0, 0]])
            self.__R = np.matrix([[0.01,0],[0,0.01]])
                                
    def predict(self):
        '''
        This is a projection step. we predict into the future.
        '''

        self.__x = self.__F * self.__x
        self.__P = (self.__F * self.__P * self.__F.T) + self.__Q
        
    def update(self,measures, auv_positions, bool):

        # Return state estimated
        [xt, yt, dotx, doty] = state_vector_to_scalars(self.__x)
        y_tilde = []
        # Compute the output error for both measuraments.
        if bool == False:
            tmp = auv_positions[0]
            tmp2 = measures - atan2(yt - auv_positions[1], xt - auv_positions[0])
            if tmp2 >= pi:
                tmp2 = tmp2 - 2*pi
            if tmp2 < -pi:
                tmp2 = tmp2 + 2*pi
            y_tilde = np.array([tmp2])
        else:
            for i in range(len(auv_positions)):
                tmp = auv_positions[i]

                y_tilde1 = measures[i] - atan2(yt - tmp[1],xt - tmp[0])
                if y_tilde1 >= pi:
                    y_tilde1 = y_tilde1 - 2*pi
                if y_tilde1 < -pi:
                    y_tilde1 = y_tilde1 + 2*pi
                y_tilde.append(y_tilde1)
            y_tilde = np.array([[y_tilde[0]], [y_tilde[1]]])
    
        self.recompute_H(auv_positions, bool)

        # Pre compute for the kalman gain K

        S = self.__H * self.__P * self.__H.T + self.__R

        K = self.__P*self.__H.T*np.linalg.inv(S)

        #Update our prediction using the error and kalman gain.
        
        self.__x = self.__x + K*y_tilde
        self.__P = (self.__xI - K*self.__H) * self.__P

