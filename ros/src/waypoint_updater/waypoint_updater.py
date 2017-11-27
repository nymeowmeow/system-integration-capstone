#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from util import Util

import math
import tf
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
KMPH = 0.2778
MPH=0.44704
STOP_DISTANCE = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb,queue_size=1)  #45-50Hz
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)  #once 
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1) #10Hz
        rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', 
						Lane, queue_size=1)

	self.waypoints          = None #waypoints of the track, 10902 waypoints
	self.pose               = None #current position
        self.prev_waypoint      = 0
	self.rate               = rospy.Rate(10)
	self.speed              = rospy.get_param('/waypoint_loader/velocity', 10)*KMPH
	self.seq                = 0    #sequence number
	self.frame_id           = '/world'
        self.red_light_waypoint = None
	self.waypoint_len       = None
	self.distance_threshold = 10   #10m
	self.max_decl           = 1

	self.processLoop()

    def processLoop(self):
	#rospy.spin()
	while not rospy.is_shutdown():
	    self.rate.sleep()

	    if not (self.pose and self.waypoints):
		#wait until we received the initial message
		continue

	    start_pos = self.next_waypoint()
	    #if (start_pos % 100) == 0:
	        #rospy.logerr('start pos: {}'.format(start_pos))
	    end = start_pos + LOOKAHEAD_WPS
	    if end > len(self.waypoints):
		endpos = end - len(self.waypoints) 
		waypoints = self.waypoints[start_pos:] + self.waypoints[:endpos]
	    else:
	        waypoints = self.waypoints[start_pos:start_pos + LOOKAHEAD_WPS]
	    if self.stopOnRedLight():
		red_index = max(0, self.red_light_waypoint - start_pos)
		redpose = waypoints[red_index]
		redpose.twist.twist.linear.x = 0

		for i, wp in enumerate(waypoints):
		    if i > red_index:
			speed = 0
		    else:
			dist = Util.distance(wp.pose.pose.position, redpose.pose.pose.position)
			dist = max(0, dist - STOP_DISTANCE)
			speed = np.sqrt(2 * self.max_decl * dist)
			if speed < 1.0:
			    speed = 0
		    wp.twist.twist.linear.x = min(speed, wp.twist.twist.linear.x, self.speed)
	    else:
	        for idx in range(len(waypoints)):
		    self.set_waypoint_velocity(waypoints, idx, self.speed)

	    #publish waypoints
	    self.prev_waypoint = start_pos
 	    self.publish_waypoint(waypoints)

    def stopOnRedLight(self):
	if self.red_light_waypoint is None or self.waypoints is None or \
		self.red_light_waypoint < 0:
		return False
	if self.red_light_waypoint >= len(self.waypoints):
	    rospy.logerr("invalid red light waypoint, out of range")

	d = Util.distance(self.pose.position, self.waypoints[self.red_light_waypoint].pose.pose.position)
	if d <= 2*STOP_DISTANCE and self.aheadOf(self.waypoints[self.red_light_waypoint], self.pose):
	    return True
	return False

    def aheadOf(self, wp1, pose):
	roll, pitch, yaw = tf.transformations.euler_from_quaternion([ pose.orientation.x, pose.orientation.y,
		pose.orientation.z, pose.orientation.w])
	xy_vector = (wp1.pose.pose.position.x - pose.position.x, wp1.pose.pose.position.y - pose.position.y)
	yaw_vector = (math.cos(yaw), math.sin(yaw)) 
	return np.inner(xy_vector, yaw_vector) > 0
 
    def next_waypoint(self):
	min_dist = float("inf")
	index = 0
	if not self.waypoint_len:
	    self.waypoint_len = len(self.waypoints)
	isfullsearch = False
	for i in range(self.waypoint_len):
	    d = Util.distance(self.pose.position, 
			self.waypoints[(i + self.prev_waypoint)%self.waypoint_len].pose.pose.position)
	    if d < min_dist:
		min_dist = d
                index = i
	    elif not isfullsearch:
		if min_dist < self.distance_threshold:
		    break
		else:
		    #ego car may be behind us, need to do full search
		    isfullsearch = True

	index = (index + self.prev_waypoint)%self.waypoint_len
	if not self.aheadOf(self.waypoints[index], self.pose):
	    index = (index+1)%self.waypoint_len
	    
	return index

    def publish_waypoint(self, waypoints):
	msg = Lane()
	msg.header.seq = self.seq
	self.seq += 1
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = self.frame_id
	msg.waypoints = waypoints
	self.final_waypoints_pub.publish(msg)

    def pose_cb(self, msg):
	self.pose = msg.pose

    def waypoints_cb(self, waypoints):
	if self.waypoints is None:
	    self.waypoints = waypoints.waypoints
	    self.frame_id  = waypoints.header.frame_id

    def traffic_cb(self, msg):
       self.red_light_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
