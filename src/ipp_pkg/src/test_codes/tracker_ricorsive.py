import os
import importlib.util
import time
spec = importlib.util.spec_from_file_location("module.est", "/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes/estimation_ricorsive.py")
est = importlib.util.module_from_spec(spec)
spec.loader.exec_module(est)
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs')

class Tracker:
    '''
    The Tracker class is created everytime we detect a target.
    It contains the entire state of the tracked object.
    '''
    def __init__(self, id, bool, init_state_vector):
        self.bool = bool
        self.__estimator = est.Estimator(bool,id, init_state_vector)
        self.id  = id
        self.__timestamp = 0
        self.__curr_time = 0
        self.__prev_time = 0

        


    @property
    def state(self):
        return self.__estimator.current_estimate

    def processMeasurement(self, table_row): #table = [tempo, misura, auv pos x, auv pos y]
        meas_data = table_row
        if self.bool == True: # for optimization node 
            self.__estimator.iteration(meas_data[0], meas_data[1], meas_data[2], meas_data[3], 0)
        else: 
            self.__estimator.iteration(meas_data[0], meas_data[1], meas_data[2], meas_data[3], self.__timestamp)
        self.__timestamp = meas_data[0]

    def propagate_estimation(self, curr_time):
        self.__curr_time = curr_time
        self.__estimator.propagation(self.__curr_time, self.__prev_time)
        self.__prev_time = self.__curr_time


        
