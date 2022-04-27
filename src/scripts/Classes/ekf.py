import numpy as np
import numpy.matlib
from math import atan2, sin, cos, pi
import scipy as sp
import control as ct
import control.optimal as opt
import control.flatsys as flat
from control.tests.conftest import slycotonly

def cost_function(sys, P0, H, u0=0, x0=0):


    S = H * P0 * H.T #+ R

    K = P0*H.T*np.linalg.inv(S)

    Pk = P0 - K*H*P0

    if Q is not None:
        Q = np.atleast_2d(Q)
        if Q.size == 1:         # allow scalar weights
            Q = np.eye(sys.nstates) * Q.item()
        elif Q.shape != (sys.nstates, sys.nstates):
            raise ValueError("Q matrix is the wrong shape")

    if R is not None:
        R = np.atleast_2d(R)
        if R.size == 1:         # allow scalar weights
            R = np.eye(sys.ninputs) * R.item()
        elif R.shape != (sys.ninputs, sys.ninputs):
            raise ValueError("R matrix is the wrong shape")




    if Q is None:
        return lambda x, u: ((u-u0) @ R @ (u-u0)).item()

    if R is None:
        return lambda x, u: ((x-x0) @ Q @ (x-x0)).item()

    # Received both Q and R matrices
    return lambda x, u: ((x-x0) @ Q @ (x-x0) + (u-u0) @ R @ (u-u0)).item()



def vehicle_update(t, x, u, params):
    # Get the parameters for the model

    # Return the derivative of the state
    return np.array([
        cos(x[2]) * u[0],            # xdot = cos(theta) vlin
        sin(x[2]) * u[0],            # ydot = sin(theta) vlin
        u[1]       # thdot = deltaTheta
    ])


def vehicle_output(t, x, u, params):
    
    return x 

def state_vector_to_scalars(state_vector):
    '''
    Returns the elements from the state_vector as a tuple of scalars.
    '''

    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0],state_vector[3][0,0])    

class ExtendedKalmanFilter:
    def __init__(self):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        '''
        self.__x = None
        self.__F = None
        self.__Q = None

        self.__P = np.matrix([[1,0,0,0],
                              [0,1,0,0],
                              [0,0,1,0],
                              [0,0,0,1]])

        self.__H = np.matlib.zeros((1,4))

        self.__R = np.matrix([[1]])
        self.__B = np.matrix([1],[1])
        self.__K1 = self.__R + self.__B
        #This is for adding disturbance on the target model
        self.__noise_ax = 0
        self.__noise_ay = 0
        self.trackingDataState = []
        self.count = 0

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
        e24 = dt3 * self.__noise_ay /  2
        e31 = dt3 * self.__noise_ax / 2
        e33 = dt2 * self.__noise_ax
        e42 = dt3 * self.__noise_ay / 2
        e44 = dt2 * self.__noise_ay

        self.__Q = np.matrix([[e11, 0, e13, 0],
                              [0, e22, 0, e24],
                              [e31, 0, e33, 0],
                              [0, e42, 0, e44]])

    def recompute_HR(self, sensor_state):

        
        px,py, vx, vy = state_vector_to_scalars(self.__x)
        #calculate_jacobian of the current state.
        sensor_state1 = sensor_state
        
        
        rx1 = px - sensor_state1[0]
        ry1 = py - sensor_state1[1]
        #print(rx1,ry1)
        self.__H = np.matrix([[-ry1/(ry1**2+rx1**2), rx1/(rx1**2+ry1**2) , 0, 0]])
                                
    def predict(self):
        '''
        This is a projection step. we predict into the future.
        '''

        self.__x = self.__F * self.__x
        self.__P = (self.__F * self.__P * self.__F.T) + self.__Q # Q INFLUISCE SU P
        
    def update(self,measure, sensor_state, target, vehicle_pose):

        [xt, yt, dotx, doty] = state_vector_to_scalars(self.__x)
        s1 = sensor_state
        
        y_tilde = measure - atan2(yt - s1[1], xt - s1[0]) # rispetto al mondo, non è il bearing ma è la pos ang
        self.recompute_HR(sensor_state)

        #pre compute for the kalman gain K
        #TODO: this code is not DRY should refactor here.
        S = self.__H * self.__P * self.__H.T + self.__R

        K = self.__P*self.__H.T*np.linalg.inv(S)

        #now we update our prediction using the error and kalman gain.
        
        self.__x = self.__x + K*y_tilde
        self.__P = self.__P - K*self.__H*self.__P
        self.trackingDataState.append(self.__x)

        #np.savetxt('trackedState.txt',self.trackingDataState[], fmt='%2f')
