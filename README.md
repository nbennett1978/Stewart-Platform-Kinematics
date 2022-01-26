# Stewart-Platform-Kinematics

This script performs the forward kinematics of stewart platform using an
algorithm that employs newton raphson methods, inverse kinematics and screws.

The details of the algorithm can be found on 
http://ieeexplore.ieee.org/abstract/document/147884/citations?tabFilter=papers

This project was forked from https://github.com/vinaym815/Stewart-Platform-Kinematics

The pupose of this fork is to create a function that takes the joint coordinates of base 
frame and end-effector frame, and strut lengths as input and calculates the homogeneous transform 
for transformation from the end-effector frame to the base frame
