#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math
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

LOOKAHEAD_WPS = 50	#200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1

        # for debug:
        self.closest_idx = -1

        #rospy.spin()
        # To control the publishing frequency
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                self.publish_waypoints()
            rate.sleep()    

    def get_closest_waypoint_idx(self):
     	# car coordinates
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        # check if closest is befor or after the car
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        # if the closet is behind the car, take the next index
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        return closest_idx


    def decelerate_waypoints(self, waypoints, closest_idx):
        # create new msg type, preserve base waypoints
        temp = []
        
        # stop 2 wp before the stop line 
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
        
        # enumerate over a sliced array of waypoints
        for i, wp in enumerate(waypoints):
            # create a new waypoint msg
            p = Waypoint()
            # the position of the new wp is not changed
            p.pose = wp.pose

            # calc distance to the stop
            dist = self.distance(waypoints, i, stop_idx)

            # velocity decelerates by square root function profile
            vel = math.sqrt(2 * MAX_DECEL * dist)
            # decelerate to zero
            if vel < 1.:
                vel = 0.

            #rospy.loginfo('WU: dist[{}]={} vel={}'.format(i, dist, vel))

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

            #rospy.loginfo('WU: twist[{}]={} vel={}'.format(i, p.twist.twist.linear.x, vel))

            temp.append(p)

        # rospy.loginfo('WU: closest_idx={} stop_idx={}'.format(self.closest_idx, stop_idx))
        # rospy.loginfo('WU: {}'.format(temp))
        # rospy.loginfo('\n')

        return temp


    def publish_waypoints(self):
        lane = self.get_lane()
        self.final_waypoints_pub.publish(lane)

        #rospy.loginfo('WU: twist={}'.format(lane.waypoints[0].twist.twist.linear.x))
        #for i, wp in enumerate(lane.waypoints):
            #rospy.loginfo('WU: twist[{}]={}'.format(i, wp.twist.twist.linear.x))
        #pass

    def get_lane(self):
        # create new lane
        lane = Lane()        
        # the header is the same
        lane.header = self.base_waypoints.header

        # slice waypoints from closest index to the end
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        # debug
        self.closest_idx = closest_idx

        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            # no TL data - publish base waypoints
            lane.waypoints = base_waypoints
        else:
            # decelerate to stop before TL
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def pose_cb(self, msg):
    	# TODO: Implement
        self.pose = msg
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # save waypoints
        self.base_waypoints = waypoints
        # make sure waypoints_2d is initialized before subsrciber callback is called 
        if not self.waypoints_2d:
        	# convert waypoints to 2D coordinates
        	self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        	# construct the KDTree to find closest waypoint
        	self.waypoint_tree = KDTree(self.waypoints_2d)
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        #rospy.loginfo('WU: stopline_wp_idx={} closest_idx={}'.format(self.stopline_wp_idx, self.closest_idx))
        #pass

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

    def kph_to_mps(self, kph):
        return 0.278 * kph

    def mph_to_mps(self, mph):
        return 0.447 * mph


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
