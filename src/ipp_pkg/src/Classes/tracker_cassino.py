import os
import importlib.util
import time
spec = importlib.util.spec_from_file_location("module.dekf", "/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes/cassino.py")
dekf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dekf)
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs')

class Tracker:
    '''
    The Tracker class is created everytime we detect a target.
    It contains the entire state of the tracked object.
    '''
    def __init__(self, id, bool, phi=0, y=0):
        self.bool = bool
        if self.bool == True:
            self.__estimator = dekf.Estimator(bool, phi, y)
        else:
            self.__estimator = dekf.Estimator(bool)
        self.id  = id
        self.__is_initialized = False
        self.__curr_time = 0
        self.__prev_time = 0

    @property
    def state(self):
        return self.__estimator.current_estimate

    def processMeasurement(self, table): #table = [tempo, misura, auv pos x, auv pos y]
        
        if self.bool == True:
            self.__prev_time = 0
            for i in range(len(table)):
                tmp = table[i]
                self.__estimator.iteration(tmp[0], tmp[1], tmp[2], tmp[3], self.__prev_time)

            #for i in range(len(table)):
            #    
            #    tmp = table[i]
            #    if self.__is_initialized:
            #        self.__estimator.propagation(tmp[0], self.__curr_time, self.__prev_time)
        else:
            for i in range(len(table)):
                meas_data = table[i]
                self.__estimator.iteration(meas_data[0], meas_data[1], meas_data[2], meas_data[3], self.__prev_time)

    def propagate_estimation(self, curr_time):

        self.__curr_time = curr_time
        self.__estimator.propagation(self.__curr_time, self.__prev_time)
        self.__prev_time = self.__curr_time


        
