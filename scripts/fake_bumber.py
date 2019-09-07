#!/usr/bin/env python

import rospy
from utils import Turtle
from utils import World
from math import cos, sin
from turtlesim_examples.msg import Bumber

rospy.init_node('bumber_node')
world = World()

pub = rospy.Publisher('bumber', Bumber, queue_size=10)
msg = Bumber()
name = rospy.get_param('~turtle_name', default='turtle1')
radius = rospy.get_param('~radius', default=1.0)
turtle1 = Turtle(name)
theta = 0.

x = turtle1.get_pose()[0][0] + radius*cos(theta)
y = turtle1.get_pose()[0][0] + radius*sin(theta)

try:
    turtle_scanner = world.spawn('turtle_scanner', x, y)
except:
    turtle_scanner = Turtle('turtle_scanner')
turtle_scanner.set_pen(1)

diff = 0.5

while not rospy.is_shutdown():
    theta += diff
    if theta > 1.5:
        diff = -0.3
    if theta < -1.5:
        diff = 0.3
    
    x = turtle1.get_pose()[0][0] + radius*cos(theta+turtle1.get_pose()[2][0])
    y = turtle1.get_pose()[1][0] + radius*sin(theta+turtle1.get_pose()[2][0])
    turtle_scanner.set_pose(x,y)
    if turtle_scanner.sense_color() == [0,0,0]:
        msg.theta = theta+turtle1.get_pose()[2][0]
        msg.distance = radius
        pub.publish(msg)


    if world.check(turtle1.get_pose()[0][0],turtle1.get_pose()[1][0]) == 100:
        rospy.logwarn("turtle crashed into wall")

    
