cmake_minimum_required(VERSION 2.8.3)
project(aerial_global_planner)
find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    nav_msgs)

catkin_package()
