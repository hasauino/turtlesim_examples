#!/usr/bin/env python

import rospy
from utils import Turtle
from utils import World
from math import cos, sin
from sensor_msgs.msg import LaserScan
from time import time

rospy.init_node('laser_node')


pub = rospy.Publisher('scan', LaserScan, queue_size=10)


name = rospy.get_param('~turtle_name', default='turtle1')
map_tc = rospy.get_param('~map_topic', default='/map')
range_max = rospy.get_param('~range_max', default=3.0)
range_min = rospy.get_param('~range_min', default=0.0)
angle_min = rospy.get_param('~angle_min', default=-1.5)
angle_max = rospy.get_param('~angle_max', default=1.5)
frame_id = rospy.get_param('~frame_id', default='base_link')
linear_increment = rospy.get_param('~linear_increment', default=0.1)
angle_increment = rospy.get_param('~angle_increment', default=0.01)
freq = rospy.get_param('~rate', default=100)
rate = rospy.Rate(freq)


world = World(map_topic=map_tc)
turtle1 = Turtle(name)


msg = LaserScan()
msg.angle_min = angle_min
msg.angle_max = angle_max
msg.angle_increment = angle_increment
msg.range_max = range_max
msg.range_min = range_min
msg.time_increment = 0.001
msg.range_min = range_min
msg.header.frame_id = frame_id
start = time()
while not rospy.is_shutdown():
    scan_time = time()-start
    start = time()
    ranges = []
    theta = angle_min
    while theta <= angle_max:
        distance = range_min
        while distance <= range_max:
            x = turtle1.get_pose()[0][0] + distance*cos(theta+turtle1.get_pose()[2][0])
            y = turtle1.get_pose()[1][0] + distance*sin(theta+turtle1.get_pose()[2][0])
            if world.check(x,y) == 100:
                break
            distance += linear_increment
        ranges.append(distance)
        theta += angle_increment
    msg.ranges = ranges
    msg.header.stamp = rospy.Time.now()
    msg.header.seq += msg.header.seq
    msg.scan_time = scan_time
    pub.publish(msg)

    if world.check(turtle1.get_pose()[0][0],turtle1.get_pose()[1][0]) == 100:
        rospy.logwarn("turtle crashed into wall")
    rate.sleep()

    
