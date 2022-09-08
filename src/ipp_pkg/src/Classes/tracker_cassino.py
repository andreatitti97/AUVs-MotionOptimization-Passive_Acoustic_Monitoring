import os
import importlib.util
spec = importlib.util.spec_from_file_location("module.dekf", "/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes/cassino.py")
dekf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dekf)
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs')

class Tracker:
    '''
    The Tracker class is created everytime we detect a target.
    It contains the entire state of the tracked object.
    '''
    def __init__(self, id, n_auv, bool, phi=0, y=0):
        self.bool = bool
        if self.bool == True:
            self.__estimator = dekf.Estimator(n_auv, bool, phi, y)
        else:
            self.__estimator = dekf.Estimator(n_auv, bool)
        self.id  = id
        self.n_auv = n_auv
        self.__is_initialized = False
        self.__curr_time = 0 

    @property
    def state(self):
        return self.__estimator.current_estimate

    def processMeasurement(self, table, curr_time): #table = [tempo, misura, auv pos x, auv pos y]
        
        self.__curr_time = curr_time
        if self.bool == True:
            for i in range(len(table)):
                tmp = table[i]
                self.__estimator.iteration(tmp[0], tmp[1], tmp[2], tmp[3],self.__curr_time)

            for i in range(len(table)):
                tmp = table[i]
                if self.__is_initialized:
                    self.__estimator.propagation(tmp[0], self.__curr_time)
        else:
            self.__estimator.iteration(table[0], table[1], table[2], table[3], self.__curr_time)
            for i in range(len(table)):
                tmp = table[i]
                if self.__is_initialized:
                    self.__estimator.propagation(table[0], self.__curr_time)


        
