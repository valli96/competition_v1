import time

class PID_controller:
    def __init__(self, kp_v_x,  kp_v_y, kp_v_z, kp_av, ki_v_x, ki_v_y, ki_v_z, ki_av, kd_v_x,
                 kd_v_y, kd_v_z, kd_av, ki_v_x_min, ki_v_y_min, ki_v_z_min, ki_av_min,
                 ki_v_x_max, ki_v_y_max, ki_v_z_max, ki_av_max):

        self._kp_v_x = kp_v_x
        self._kp_v_y = kp_v_y
        self._kp_v_z = kp_v_z
        self._kp_av = kp_av

        self._ki_v_x = ki_v_x
        self._ki_v_y = ki_v_y
        self._ki_v_z = ki_v_z
        self._ki_av = ki_av

        self._kd_v_x = kd_v_x
        self._kd_v_y = kd_v_y
        self._kd_v_z = kd_v_z
        self._kd_av = kd_av

        self._ki_v_x_min = ki_v_x_min
        self._ki_v_y_min = ki_v_y_min
        self._ki_v_z_min = ki_v_z_min
        self._ki_av_min = ki_av_min

        self._ki_v_x_max = ki_v_x_max
        self._ki_v_y_max = ki_v_y_max
        self._ki_v_z_max = ki_v_z_max
        self._ki_av_max = ki_av_max

        self._linear_controler_x = 0
        self._linear_controler_y = 0
        self._linear_controler_z = 0
        self._angular_controler = 0

        self.dt = None
        self._last_time = None
        self._linear_error_x_last = None
        self._linear_error_y_last = None
        self._linear_error_z_last = None
        self._angular_error_last = None

    def calcualte_time_loop(self):
        """
        start loop timing and calculate the time of one loop
        first_loop -> dt = 0
        """
        actual_time = time.time()

        if self._last_time == None:
            self._last_time = actual_time

        self.dt = actual_time - self._last_time
        self._last_time = actual_time

    def update_PID_contoller(self, linear_error_x, linear_error_y, linear_error_z, angular_error):
        """
        linear_error:= x and y difference between actual position and goal position
        angular_error:= angular difference between actual angel and the angel to the goal

        The actual implementation  of the PID controller.

        """
        self.calcualte_time_loop()
        if self.dt == 0:
            self._linear_controler_x = 0
            self._linear_controler_y = 0
            self._linear_controler_z = 0
            self._angular_controler = 0
            return self._linear_controler_x, self._linear_controler_y, self._linear_controler_z, self._angular_controler,

        # for the first round
        if(self._linear_error_x_last == None):
            self._linear_error_x_last = linear_error_x
        if(self._linear_error_y_last == None):
            self._linear_error_y_last = linear_error_y
        if(self._linear_error_z_last == None):
            self._linear_error_z_last = linear_error_z

        if(self._angular_error_last == None):
            self._angular_error_last = angular_error

        # P controller part
        linear_velocity_p_x = self._kp_v_x * linear_error_x
        linear_velocity_p_y = self._kp_v_y * linear_error_y
        linear_velocity_p_z = self._kp_v_z * linear_error_z
        angular_velocity_p = self._kp_av * angular_error

        # D controller part
        linear_velocity_d_x = self._kd_v_x * \
            (self._linear_error_x_last - linear_error_x)/self.dt
        linear_velocity_d_y = self._kd_v_y * \
            (self._linear_error_y_last - linear_error_y)/self.dt
        linear_velocity_d_z = self._kd_v_z * \
            (self._linear_error_z_last - linear_error_z)/self.dt

        # angular_velocity_d = self._kd_av * \
        #     (self._angular_error_last - angular_error)/self.dt
        
        linear_velocity_i_z = self.dt*(self._linear_error_z_last - linear_error_z)*self._ki_v_z
        
        self._linear_error_x_last = linear_error_x
        self._linear_error_y_last = linear_error_y
        self._linear_error_z_last = linear_error_z
        self._angular_error_last = angular_error

       
        self._linear_controler_x = linear_velocity_p_x + linear_velocity_d_x
        self._linear_controler_y = linear_velocity_p_y + linear_velocity_d_y
        self._linear_controler_z = linear_velocity_p_z + linear_velocity_d_z + linear_velocity_i_z
        self._angular_controler = angular_velocity_p  # + angular_velocity_d