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
    def __init__(self, id, n_auv):

        self.id  = id
        self.n_auv = n_auv
      
        self.__ekf = dekf.Estimator(n_auv)
        self.__is_initialized = False
        self.__curr_time = 0 

    @property
    def state(self):
        return self.__ekf.current_estimate

    def processMeasurement(self,target_init, table, curr_time): #table = [misura, tempo, auv pos x, auv pos y]
        # if this is initialization with the first measurament for setup state vector.
        if not self.__is_initialized: # non puoi propagare se non hai lo stato iniziale, ora, su cassino non è ben chiaro come
                                            #si inizializza lo stato, io rimango come con ekf, fornisco un guess iniziale con disturbo.
          
            vx, vy = target_init[2], target_init[3]
            x0, y0 = target_init[0], target_init[1]
            self.__ekf.init_state_vector(x0,y0, vx, vy)
            #self.__previous_timestamp = measurement_packet.timestamp
            
            self.__is_initialized = True
            return
        self.__curr_time = curr_time
        for i in range(len(table)):
            tmp = table[i]
            print(tmp)
            print(tmp[0])
        # Non c'è piu un dt fisso, ma curr time stamps and measu timestamp
        # Set new F and Q using new dt
            self.__ekf.iteration(tmp[0], tmp[1], tmp[2], tmp[3],self.__curr_time)
        
        for i in range(len(table)):
            tmp = table[i]
            print(tmp)
            print(tmp[0])
            if self.__is_initialized:

                self.__ekf.propagation(tmp[0], self.__curr_time)


        
