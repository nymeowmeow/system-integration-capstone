import rospy
import math
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
	self.vehicle_mass     = args[0]
	self.fuel_capacity    = args[1]
	self.brake_deadband   = args[2]
	self.decel_limit      = args[3]
	self.accel_limit      = args[4]
	self.wheel_radius     = args[5]
	self.wheel_base       = args[6]
	self.steer_ratio      = args[7]
	self.max_lat_accel    = args[8]
	self.max_steer_angle  = args[9]
	self.max_throttle     = args[10]

	self.last_dbw_enabled = None
	self.pid_velocity     = PID(0.2, 0.08, 0.01, self.decel_limit, self.max_throttle)
	#self.pid_steering     = PID(0.01, 0.0, 0.05, -self.max_steer_angle, self.max_steer_angle)
	self.pid_steering     = PID(0.85, 0.0, 0.25, -self.max_steer_angle, self.max_steer_angle) #0.85, 0.005, 0.25
	self.accl_filter      = LowPassFilter(4, 1)
	self.yaw_controller   = YawController(self.wheel_base, self.steer_ratio,
					0, self.max_lat_accel,
					self.max_steer_angle)
	self.steer_filter     = LowPassFilter(0.2, 1.0)

    def control(self, *args, **kwargs):
	target_velocity  = args[0]
	target_steer     = args[1]
	current_velocity = args[2]
	current_steer    = args[3]
	is_dbw_enabled   = args[4]
	elapsed          = args[5]
	current_steering = args[6] if args[6] else 0
	cte              = args[7]

	#if abs(current_steering < 0.001):
	    #current_steering = 0

	if self.last_dbw_enabled  != is_dbw_enabled and is_dbw_enabled:
	    self.pid_velocity.reset()
	    self.pid_steering.reset()

	self.last_dbw_enabled = is_dbw_enabled

	vel_err = target_velocity - current_velocity
	accl    = self.pid_velocity.step(vel_err, elapsed)
	accl    =  self.accl_filter.filt(accl)

	steer = self.yaw_controller.get_steering(target_velocity, 
			target_steer, current_velocity)
	pred = steer
	delta = 0
	#if cte is not None:
	    #delta = self.pid_steering.step(cte, elapsed)
	    #steer += delta
	    #rospy.logerr('pred: {} steer {}, delta: {}'.format(pred, steer, delta))
	steer = self.pid_steering.step(steer - current_steering, elapsed)
	steer = self.steer_filter.filt(steer)
	#rospy.logerr('pred: {}, steer {}, current steering {}'.format(pred, steer, current_steering))
	steer = max(min(self.max_steer_angle, steer), -self.max_steer_angle)
	if (target_velocity < 0.01):
	    throttle = 0
	    brake = self.getTorque(self.decel_limit)
	else:
	    throttle = max(accl, 0)
	    brake = self.getTorque(accl) if accl < 0 else 0.0
	if brake > 0 and brake < self.brake_deadband:
	    brake = 0 
        return throttle, brake, steer

    def getTorque(self, accl):
	return abs(accl)*(self.vehicle_mass + self.fuel_capacity*GAS_DENSITY)*self.wheel_radius
 
