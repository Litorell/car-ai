
import numpy as np


class Car:
    def __init__(self):
        
        # Car dimensions 
        self._wheel_base = 3.6
        self._wheel_radius = 0.32
        self._width = 1.8
        self._length = 4.8

        # Powertrain
        self._gear_ratios = 6 / np.arange(1, 6+1) * 2
        self._max_rpm = 8000 # rad/s
        self._max_torque = 500 # Nm
        # self._powertrain_inertia = 10 # kgm^2
        # self._wheel_inertia = 2 # kgm^2

        # Dynamics
        self._mass = 1500

        # Aerodynamics
        self._frontal_area = 2.2
        self._drag_coefficient = 0.4

        # Brakes
        self._max_braking_torque = 4000 # Nm
        # self._brake_bias = 0.5


        # State variables
        self._pos = np.array([0.0, 0.0])
        self._vel = np.array([0.0, 0.0])
        self._acc = np.array([0.0, 0.0])
        
        self._yaw = 0
        self._yaw_rate = 0

        # self._wheel_speed = [0, 0, 0, 0] # FL, FR, RL, RR
        self._motor_force = 0
        
        # Driver inputs
        self._steering_angle = 0
        self._throttle = 0
        self._brake = 0
        self._gear = 0

        # Traction
        # self._static_friction_coefficient = 0.8
        # self._kinetic_friction_coefficient = 0.6
        # self._slip_velocity_threshold = 1 # m/s
    
    def __get_wheel_local_position(self, wheel_index):
        """Returns the position of the wheel center in the car frame."""
        return [
            np.array([self._wheel_base/2, self._width/2]),
            np.array([self._wheel_base/2, -self._width/2]),
            np.array([-self._wheel_base/2, self._width/2]),
            np.array([-self._wheel_base/2, -self._width/2])
        ][wheel_index]

    def rotate90(self, vec):
        return np.array([-vec[1], vec[0]])
    

    def get_wheel_turning_angle(self, wheel_index):
        """Returns the angle of the wheel in the world frame based on the steering angle."""
        if wheel_index in [0, 1]:
            return self._steering_angle + self._yaw # TODO: Account for Ackermann steering
        return self._yaw
        


    def get_wheel_center_velocity(self, wheel_index):
        """Returns the velocity of the wheel center in the world frame."""
        wheel_local_position = self._get_wheel_local_position(wheel_index)
        wheel_local_velocity = self._vel + self.rotate90(wheel_local_position) * self._yaw_rate
        return wheel_local_velocity
        
        
    def get_wheel_contact_point_velocity(self, wheel_index):
        """Returns the velocity of the wheel contact point in the world frame."""
        wheel_local_position = self._get_wheel_local_position(wheel_index)
        wheel_local_velocity = self._vel + self.rotate90(wheel_local_position) * self._yaw_rate
        return wheel_local_velocity + self.rotate90(wheel_local_position) * self._wheel_speed[wheel_index]

    def get_rotation_matrix(self):
        return np.array([
            [np.cos(self._yaw), -np.sin(self._yaw)],
            [np.sin(self._yaw), np.cos(self._yaw)]
        ])
    
    def update(self, dt):
        
        local_vel = self.get_rotation_matrix().T @ self._vel
        lat_vel = local_vel[0]
        long_vel = local_vel[1]

        # Steering
        a_lat = np.tan(self._steering_angle) * long_vel**2 / self._wheel_base

        # Motor
        motor_rpm = self.motor_rpm
    
        if motor_rpm > self._max_rpm:
            self._motor_force = 0
        else:
            motor_torque = self._throttle * self._max_torque
            self._motor_force = motor_torque / self._wheel_radius * self._gear_ratios[self._gear]

        # Brakes
        brake_force = -self._brake * self._max_braking_torque / self._wheel_radius * np.sign(long_vel)

        # Aerodynamics
        drag_force = -0.5 * self._drag_coefficient * self._frontal_area * long_vel**2 * np.sign(long_vel)
        a_lon = (self._motor_force + brake_force + drag_force) / self._mass

        
        self._yaw_rate = long_vel / self._wheel_base * np.tan(self._steering_angle)

        local_acc = np.array([a_lat, a_lon])
        self._acc = self.get_rotation_matrix() @ local_acc

        self._vel += self._acc * dt
        self._pos += self._vel * dt

        self._yaw += self._yaw_rate * dt

        
    @property
    def pos(self):
        return self._pos
        
    @property
    def vel(self):
        return self._vel
    
    @property
    def acc(self):
        return self._acc
    
    @property
    def yaw(self):
        return self._yaw
    
    @property
    def yaw_rate(self):
        return self._yaw_rate
    
    @property
    def steering_angle(self):
        return self._steering_angle
    
    @steering_angle.setter
    def steering_angle(self, value):
        value = np.clip(value, -1, 1) * np.pi / 4
        self._steering_angle = value

    @property
    def throttle(self):
        return self._throttle
    
    @throttle.setter
    def throttle(self, value):
        self._throttle = np.clip(value, 0, 1)

    @property
    def brake(self):
        return self._brake
    
    @brake.setter
    def brake(self, value):
        self._brake = np.clip(value, 0, 1)

    @property
    def gear(self):
        return self._gear
    
    @gear.setter
    def gear(self, value):
        self._gear = np.clip(value, 0, len(self._gear_ratios) - 1).astype(int)

    
    @property
    def motor_rpm(self):
        long_vel = (self.get_rotation_matrix().T @ self._vel)[1]
        return long_vel / self._wheel_radius * self._gear_ratios[self._gear] * 60 / (2 * np.pi)
    

    @property
    def max_rpm(self):
        return self._max_rpm
    

    @property
    def max_torque(self):
        return self._max_torque
    
    @property
    def motor_force(self):
        return self._motor_force







