# This plugin corresponds to the differential_system.cpp plugin from Gazebo, adapted for Webots.

class DifferentialPlugin:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__motor_L = self.__robot.getDevice('rocker_L_joint')
        self.__motor_R = self.__robot.getDevice('rocker_R_joint')

        self.__sensor_L = self.__robot.getDevice('rocker_L_joint_sensor')
        self.__sensor_R = self.__robot.getDevice('rocker_R_joint_sensor')

        self.__time_step = int(self.__robot.getBasicTimeStep())
        if self.__sensor_L:
            self.__sensor_L.enable(self.__time_step)
        if self.__sensor_R:
            self.__sensor_R.enable(self.__time_step)
            
        self.__max_torque = 100.0

        if self.__motor_L:
            self.__motor_L.setAvailableTorque(self.__max_torque)
            
        if self.__motor_R:
            self.__motor_R.setAvailableTorque(self.__max_torque)

    def step(self):
        if not self.__sensor_L or not self.__sensor_R or not self.__motor_L or not self.__motor_R:
            return

        pos_L = self.__sensor_L.getValue()
        pos_R = self.__sensor_R.getValue()
        
        avg_pos = (pos_L + pos_R) / 2.0
        
        if abs(pos_L - avg_pos) > 0.001 or abs(pos_R - avg_pos) > 0.001:
            self.__motor_L.setPosition(avg_pos)
            self.__motor_R.setPosition(avg_pos)
