# aerial_global_planner
A ROS package that implements a 3D global planner based on a series of estimated goal velocities. What makes this planner unique, is that it does not generate a path towards a fixed target, but towards a moving one. On a side note, the planner assumes that there are no obstacles between the robot and its moving target.

It is part of a ROS eco-system of packages that allow an autonomous UAV to land on a moving platform, using only a front-facing camera.
You can see a tutorial [here](http://wiki.ros.org/Tutorials/Landing%20an%20autonomous%20UAV%20on%20a%20moving%20platform%20using%20only%20a%20front%20facing%20camera).
