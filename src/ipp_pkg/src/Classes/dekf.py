import numpy as np
from math import atan2, pi
from numpy import matlib

def state_vector_to_scalars(state_vector):
    '''
    Returns the elements from the state_vector as a tuple of scalars.
    '''
    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0],state_vector[3][0,0])    
    
class ExtendedKalmanFilter:
    def __init__(self,bool,init_cov=[]):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        '''
        self.__xI = matlib.identity(4)
        self.__x = None
        self.__F = None
        self.__Q = None
        
        if bool == True:
            self.__P = init_cov
        else:
            self.__P = np.matrix([[50,0,0,0], # This are set according to the initial uncertainty choosen
                              [0,50,0,0],     # if change init uncertainty change these
                              [0,0,0.1,0],    
                              [0,0,0,0.1]])   

        self.__H = matlib.zeros((2,4))

        if bool == True:
            self.__R = np.matrix([[0.01,0],[0,0.01]])
        else:
            self.__R = np.matrix([[0.01,0],[0,0.01]]) #expected meas noise variance
        #This is for adding disturbance on the target 
        # FOR NOW WHEN THE EKF IS CALLED DURING OPTIMIZATION THERE IS NO DISTURBANCE because we receive a corrpted state (both measurmane and state)
        # and from this state + cov we simply apply the linear model obtaining ONE realizatio of the target 
        # TODO this can improved by sampling from the input distribution (state+cov) other possible target realization, through #USCENTED TRANSORM
        if bool == True:
            self.__noise_ax = 0.001 
            self.__noise_ay = 0.001
        else:
            self.__noise_ax = 0.001
            self.__noise_ay = 0.001 #0.0001
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
                              
    def recompute_H(self, s1, s2):

        px,py, vx, vy = state_vector_to_scalars(self.__x)
        #calculate_jacobian of the current state.
    
        rx1 = px - s1[0]
        ry1 = py - s1[1]

        rx2 = px - s2[0]
        ry2 = py - s2[1]

        self.__H = np.matrix([[-ry1/(ry1**2+rx1**2), rx1/(rx1**2+ry1**2) , 0, 0],   #TODO vedi se si può far qualcosa per le vel
                                [-ry2/(ry2**2+rx2**2), rx2/(rx2**2+ry2**2) , 0, 0]])
                                
    def predict(self):
        '''
        This is a projection step. we predict into the future.
        '''

        self.__x = self.__F * self.__x
        self.__P = (self.__F * self.__P * self.__F.T) + self.__Q
        
    def update(self,measures, sensor_state1, sensor_state2):

        # Return state estimated
        [xt, yt, dotx, doty] = state_vector_to_scalars(self.__x)
        
        # Compute the output error for both measuraments.
        y_tilde1 = measures[0] - atan2(yt - sensor_state1[1],xt - sensor_state1[0])
        if y_tilde1 >= pi:
            y_tilde1 = y_tilde1 - 2*pi
        if y_tilde1 < -pi:
            y_tilde1 = y_tilde1 + 2*pi
        y_tilde2 = measures[1] - atan2(yt - sensor_state2[1],xt - sensor_state2[0])
        if y_tilde2 >= pi:
            y_tilde2 = y_tilde2 - 2*pi
        if y_tilde2 < -pi:
            y_tilde2 = y_tilde2 + 2*pi
        y_tilde = np.array([[y_tilde1], [y_tilde2]])

        self.recompute_H(sensor_state1,sensor_state2)

        # Pre compute for the kalman gain K

        S = self.__H * self.__P * self.__H.T + self.__R

        K = self.__P*self.__H.T*np.linalg.inv(S)

        #Update our prediction using the error and kalman gain.
        
        self.__x = self.__x + K*y_tilde
        self.__P = (self.__xI - K*self.__H) * self.__P

