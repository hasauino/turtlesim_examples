# turtlesim_examples
A node and launch files to make ROS turtlesim do navigation (fake laser, fake bumber, draw a map)

Example 1, turtle with fake bumber (a second turtle act as a bumber):
```
roslaunch turtlesim_examples bumber.launch
```
![turtle with fake bumber](include/2.gif)


Example 1, turtle with fake laser scanner:
```
roslaunch turtlesim_examples laser.launch
```

![turtle with fake laser](include/0.gif)


Example 2, Bringing up navigation stack on the turtle:
```
roslaunch turtlesim_examples laser.launch
```
```
roslaunch turtlesim_examples moveBase.launch
```

![turtle with fake laser](include/1.gif)
