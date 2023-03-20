# MAVLink Camera Simulator

This is a simple OpenGL renderer for generating and streaming video for simulating the camera view of a virtual drone controlled via MAVLink.

The simulator only renders an image on the ground, which is enough for testing computer vision algorithms that detect and estimate the pose of apriltag or aruco markers to implement vision precision landing.

If you need something more complex (and resource intensive), check Gazebo (https://gazebosim.org).

We are using this simulator to test this precision landing implementation:
https://github.com/kripper/vision-landing-2
