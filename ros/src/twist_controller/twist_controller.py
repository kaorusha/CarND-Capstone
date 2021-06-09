import rospy
import yaw_controller
import pid
import lowpass

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.throttle = pid.PID(
            kwargs['kp'], kwargs['ki'], kwargs['kd'], kwargs['mn'], kwargs['mx'])
        self.steer = yaw_controller.YawController(*args)
        self.lowpass = lowpass.LowPassFilter(kwargs['tau'], kwargs['ts'])
        self.vehicle_mass = kwargs['vehicle_mass']
        self.decel_limit = kwargs['decel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.last_time = rospy.get_time()

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not kwargs['state']:
            self.throttle.reset()
            return 0., 0., 0.
        current_vel = self.lowpass.filt(kwargs['curr_vel'])
        steer = self.steer.get_steering(
            kwargs['linear_vel'], kwargs['angular_vel'], current_vel)

        vel_error = kwargs['linear_vel'] - current_vel
        current_time = rospy.get_time()
        delta_t = current_time - self.last_time

        throttle = self.throttle.step(vel_error, delta_t)
        brake = 0.0

        if kwargs['linear_vel'] == 0. and current_vel < self.steer.min_speed:
            throttle = 0
            brake = 700  # N*m to hold the car in place at deceleration < 5 m/s^2

        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # torque N*m

        return throttle, brake, steer
