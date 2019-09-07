#!/usr/bin/env python

import rospy

# importing needed msg classes
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.msg import Color
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
import math


# define a class named turtle
class Turtle:
    # constructor
    def __init__(self, turtle_name='turtle1'):
        """Turtle class provides interface to control the turtle and get 
           feedback
        
        Keyword Arguments:
            turtle_name {string} -- turtle's name (default: {turtle1})
        """
        # define a private variable of type Publisher (an instance of Publisher)
        self.turtle_name = turtle_name

        self.__pub = rospy.Publisher(turtle_name+'/cmd_vel',
                                     Twist, queue_size=10)
        self.__vel_msg = Twist()
        # define subscribers for pose and color topics, and assign callbacks
        rospy.Subscriber(turtle_name+'/pose', Pose,
                         callback=self.__pose_cb)

        rospy.Subscriber(turtle_name+'/color_sensor', Color,
                         callback=self.__color_cb)
        
        rospy.wait_for_service(turtle_name+'/set_pen')
        try:
            self.__set_pen = rospy.ServiceProxy(turtle_name+'/set_pen', SetPen)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed, setpen")        
        
        rospy.wait_for_service(turtle_name + '/teleport_absolute')
        try:
            self.__set_pose = rospy.ServiceProxy(turtle_name+'/teleport_absolute', TeleportAbsolute)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed, set pose")



        # private attribute to hold the turtle pose
        self.__pose = [[0.0],
                       [0.0],
                       [0.0]]
        # private attribute to hold the turtle velocity
        self.__velocity = [[0.0],
                           [0.0],
                           [0.0]]
        # private attribute to hold the color sensor of the turtle
        self.__color = [0,0,0]


    # define a private method to be used as the callback function for the pose msg
    def __pose_cb(self, msg):
        self.__pose = [[msg.x],
                       [msg.y],
                       [msg.theta]]

        self.__velocity = [[msg.linear_velocity],
                           [0.0],
                           [msg.angular_velocity]]

    # define a private method to be used as the callback function for the color msg
    def __color_cb(self, msg):
        self.__color = [msg.r, msg.g, msg.b]

    # a public method to be used by the user to get pose
    def get_pose(self):
        """get the current pose (position and orientation) of the turtle
        
        Returns:
            list (2d list) -- 3x1 list: x (m),y (m),theta (rad)
        """
        return self.__pose

    # a public method to be used by the user to get velocity
    def get_velocity(self):
        """get the current velocity of the turtle
        
        Returns:
            list 2d list -- 3x1 list: linear velocity in x (m/s), linear 
                           velocity in y (always zero), angular velocity around
                           the z (rad/s)
        """
        return self.__velocity

    # a public method to be used by the user to get color reading
    def sense_color(self):
        """returns color reading of the turtle's color sensor
        
        Returns:
            list -- [r,g,b] representing RGB values
        """
        return self.__color

    # a public method to be used by the user to set the velocity
    def set_velocity(self, v, w):
        """sends a velocity command to the turtle
        
        Arguments:
            v {float} -- linear velocity (m/s)
            w {[type]} -- angular velocity (rad/s)
        """
        self.__vel_msg.linear.x = v
        self.__vel_msg.angular.z = w
        self.__pub.publish(self.__vel_msg)
    
    def set_pen(self, width, color="normal", state='off'):
        if color == "black":
            r = 0
            g = 0
            b = 0
        elif color == "red":
            r = 255
            g = 0
            b = 0 
        elif color == "Green":
            r = 0
            g = 255
            b = 0 
        elif color == "blue":
            r = 0
            g = 0
            b = 255                                   
        elif color == "yellow":
            r = 255
            g = 255
            b = 0
        elif color == "erase":
            r = 69
            g = 86
            b = 255
        else:
            r = 179
            g = 184
            b = 255

        if state == 'off':
            off = 1
        else:
            off = 0
        self.__set_pen(r, g, b, width, off)


    def set_pose(self, x, y, theta=0):
        self.__set_pose(x, y, theta)
    

class World:
    def __init__(self, map_topic='/map'):
        self._map_received = False
        rospy.Subscriber(map_topic, OccupancyGrid, self.__map_cb)
        rospy.loginfo("waiting for map..")
        while not self._map_received and not rospy.is_shutdown():
            pass
        rospy.loginfo("map was received successfully")
        
        rospy.wait_for_service('clear')
        try:
            self.clear = rospy.ServiceProxy('clear', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed, clear")
        rospy.wait_for_service('reset')
        try:
            self.reset = rospy.ServiceProxy('reset', Empty)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed, reset")

        rospy.wait_for_service('/spawn')
        try:
                self.__spn = rospy.ServiceProxy('/spawn', Spawn)
        except rospy.ServiceException, e:
                rospy.logwarn("Service call failed, spawn")
        rospy.wait_for_service('/kill')
        
        try:
                self.__kill = rospy.ServiceProxy('/kill', Kill)
        except rospy.ServiceException, e:
                rospy.logwarn("Service call failed, kill")                
        
        self.window_width = 11.08888912
        self.window_height = 11.08888912

    def __map_cb(self,msg):
        self._map_received = True
        self.map = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin

    def check(self, x, y):
        index = math.floor((y-self.origin.position.y)/self.resolution)*self.width +\
                math.floor((x--self.origin.position.x)/self.resolution)
        if int(index) < len(self.map):
            return self.map[int(index)]
        else:
            return 100


    def spawn(self, name, x, y, theta=0):
            msg = self.__spn(x, y, theta, name)
            return Turtle(name)
    
    def kill(self, turtle_name):
        self.__kill(turtle_name)


def draw(t, start, end, world, step):
    y = start
    while y <= end:
        t.set_pen(1)
        x = 0.
        t.set_pose(x, y)
        while x <= world.window_height:
            if world.check(x, y) == 100:
                s = 'on'
            else:
                s = 'off'
            t.set_pen(math.ceil(step*50.0), color='black', state=s)
            t.set_pose(x, y)
            x += step
        y += step
