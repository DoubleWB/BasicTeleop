This is a package containing a simple teleoporation protocol consisting of two nodes, keyop.py - which publishes directions to the robot, and teleop.py, which turns those directions into motor movements for the robot.
This was a project designed to use ROS for the first time outside of a tutorial.

There are also some nodes which are designed to do simple transformation of depth data from a 3D camera into simple blobs, and follow the blob closest to it. This is used as a platform for experimentation for ways to improve the smoothness of depth data which currently has a low publishing rate.