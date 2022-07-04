import os
import importlib.util
spec = importlib.util.spec_from_file_location("module.dekf", "/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes/4_AUV/dekf.py")
dekf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dekf)
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs')


class Tracker:
    '''
    The Tracker class is created everytime we detect a target.
    It contains the entire state of the tracked object.
    '''
    def __init__(self, id, bool, init_cov=[]):

        self.id  = id
        if bool == True:
            self.__ekf = dekf.ExtendedKalmanFilter(bool,init_cov)
        else:
            self.__ekf = dekf.ExtendedKalmanFilter(bool)
        self.__is_initialized = False
        self.__previous_timestamp = 0 # for simulate delay between measuraments TODO

    @property
    def state(self):
        return self.__ekf.current_estimate

    def processMeasurement(self,measures, target_init, sensor_state1, sensor_state2,sensor_state3,sensor_state4, tc):
        # if this is initialization with the first measurament for setup state vector.
        if not self.__is_initialized:

            vx, vy = target_init[2], target_init[3]
            x0, y0 = target_init[0], target_init[1]
            self.__ekf.init_state_vector(x0,y0, vx, vy)
            #self.__previous_timestamp = measurement_packet.timestamp
            
            self.__is_initialized = True
            return

        # Passo di campionamento filtro
        dt = tc
        #2nd set new F and Q using new dt
        self.__ekf.recompute_F_and_Q(dt)
        #3rd make a prediction
        self.__ekf.predict()
        #4th Update the observation matrix and the target state
        self.__ekf.update(measures, sensor_state1, sensor_state2, sensor_state3,sensor_state4)
        #5th COMMUNICATION 
        #TODO

        
