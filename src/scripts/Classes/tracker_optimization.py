from Classes.ekf import ExtendedKalmanFilter
import multiprocessing
import time
import numpy as np
from queue import Queue
from threading import Thread

class Tracker:
    '''
    The Tracker class is created everytime we detect a target.
    It contains the entire state of the tracked object.
    '''
    def __init__(self, id, init_cov):

        self.id  = id
        self.__ekf = ExtendedKalmanFilter(init_cov)
        self.__is_initialized = False
        self.__previous_timestamp = 0.
        # Establish communication queues
        
        

    @property
    def state(self):

        return self.__ekf.current_estimate



    def processMeasurement(self,measures, target_init, sensor_state1, sensor_state2, tc):
        # if this is initialization with the first measurament for setup state vector.
        if not self.__is_initialized:

            vx, vy = target_init[2], target_init[3]
            x0, y0 = target_init[0], target_init[1]
            self.__ekf.init_state_vector(x0,y0, vx, vy)
            #self.__previous_timestamp = measurement_packet.timestamp

            self.__is_initialized = True
            return

        # Passo di campionamento streamer
        dt = tc
        #2nd set new F and Q using new dt
        self.__ekf.recompute_F_and_Q(dt)
        #3rd make a prediction
        self.__ekf.predict()
        #4th Update the observation matrix and the target state
        self.__ekf.update(measures, sensor_state1, sensor_state2)
        
        #print('trackerID:',self.id, self.__ekf.current_estimate)
        #5th COMMUNICATION 
        
        
        # Solve Optimization problem
        print(self.__ekf.__P)
        
