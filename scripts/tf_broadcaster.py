#!/usr/bin/env python  
import rospy

import tf
from utils import Turtle

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    name = rospy.get_param('~turtle_name', default='turtle1')
    link = rospy.get_param('~link', default='base_link')
    world_frame = rospy.get_param('~world_frame', default='/map')
    turtle = Turtle(name)
    rate = rospy.Rate(100)
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        x = turtle.get_pose()[0][0]
        y = turtle.get_pose()[1][0]
        theta = turtle.get_pose()[2][0]

        br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     link,
                     world_frame)
        rate.sleep()