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

        if bool == True:
            self.__ekf = dekf.Estimator(n_auv, bool, phi, y)
        else:
            self.__ekf = dekf.Estimator(n_auv, bool)
        self.id  = id
        self.n_auv = n_auv
        self.__is_initialized = False
        self.__curr_time = 0 

    @property
    def state(self):
        return self.__ekf.current_estimate

    def processMeasurement(self,target_init, table, curr_time): #table = [tempo, misura, auv pos x, auv pos y]
        
        if not self.__is_initialized: 
            vx, vy = target_init[2], target_init[3]
            x0, y0 = target_init[0], target_init[1]
            self.__ekf.init_state_vector(x0,y0, vx, vy, curr_time)
            self.__is_initialized = True
            return
        self.__curr_time = curr_time
        for i in range(len(table)):
            tmp = table[i]
            self.__ekf.iteration(tmp[0], tmp[1], tmp[2], tmp[3],self.__curr_time)
        for i in range(len(table)):
            tmp = table[i]
            if self.__is_initialized:
                self.__ekf.propagation(tmp[0], self.__curr_time)


        
