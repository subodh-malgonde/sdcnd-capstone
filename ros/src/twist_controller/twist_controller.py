from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

'''
Source: https://discussions.udacity.com/t/what-is-the-range-for-the-brake-in-the-dbw-node/412339

From discussions on Slack, I understand the brake should be calculated as:

brake = (vehicle_mass + fuel_capacity * GAS_DENSITY) * acceleration * wheel_radius

Now taking the parameter values for the simulator (dbw_sim.launch), I get, with the max acceleration allowed:

brake = (1080.0 + 0.0 * 2.858) * (-5) * 0.335 = -1809 Nm

Does it imply that I shall send the simulator a brake value between 0 and 1809? Yes
'''
class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.wheel_base = kwargs.get("wheel_base")
        self.steer_ratio = kwargs.get("steer_ratio")
        self.max_lat_accel = kwargs.get("max_lat_accel")
        self.max_steer_angle = kwargs.get("max_steer_angle")
        self.min_speed = kwargs.get("min_speed")
        self.fuel_capacity = kwargs.get("fuel_capacity")
        self.accel_limit = kwargs.get("accel_limit")
        self.decel_limit = kwargs.get("decel_limit")
        self.wheel_radius = kwargs.get("wheel_radius")
        self.vehicle_mass = kwargs.get("vehicle_mass")

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed,
                                            self.max_lat_accel, self.max_steer_angle)

        self.lowpass = LowPassFilter(1, 1)

        self.throttle_pid_controller = PID(kp=2.0, ki=0.0, kd=1.2, mn=self.decel_limit, mx=self.accel_limit)

    def control(self, *args, **kwargs):
        # Return throttle, brake, steer
        target_linear_velocity = args[0]
        target_angular_velocity = args[1]
        current_linear_velocity = args[2]
        sample_time = args[3]
        # rospy.loginfo("target_angular_velocity %.2f" % (target_angular_velocity))

        steering = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)
        steering = self.lowpass.filt(steering)

        throttle = self.throttle_pid_controller.step(target_linear_velocity - current_linear_velocity, sample_time)
        if throttle < 0:
            braking = -1*(self.vehicle_mass + self.fuel_capacity*GAS_DENSITY)*throttle*self.wheel_radius
            throttle = 0
        else:
            braking = 0
        return throttle, braking, steering