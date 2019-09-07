#!/usr/bin/env python

import rospy
from utils import Turtle
from utils import World
from utils import draw
import math
from multiprocessing import Process


def node(turtle_name, indx, step, no_turtles, world):
    x = 0.0
    y_start = indx*(world.window_height/no_turtles)
    y_end = (indx+1)*(world.window_height/no_turtles)
    turtle = world.spawn(turtle_name, x, y_start)

    draw(turtle, y_start, y_end, world, step)

    world.kill(turtle_name)

if __name__ == '__main__':
    rospy.init_node('draw')
    task = rospy.get_param('~task', default='none')
    world = World()
    step = rospy.get_param('~step_size', default=0.5)
    no_turtles = int(math.ceil(world.window_height/(step)))
    nodes = []
    for i in range(no_turtles):
        nodes.append( Process(target=node, args=('t'+str(i), i, step,
                      no_turtles, world)))

    for node in nodes:
        node.start()

    for node in nodes:
        node.join()
    
    turtle1 = Turtle('turtle1')

    if task == 'none':
        turtle1.set_pen(1)
    elif task == 'cleaning':
        turtle1.set_pen(50, 'erase', 'on')
    elif task == 'treasure':
        turtle1.set_pen(1)
        turtle1.set_pose(9,9)
        turtle1.set_pen(20, 'red', 'on')
        turtle1.set_pose(9.2,9)
        turtle1.set_pen(1)
        turtle1.set_pose(1, 1)

