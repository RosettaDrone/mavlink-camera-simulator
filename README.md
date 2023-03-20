# MAVLink Camera Simulator

This is a simple OpenGL renderer for generating and streaming video for simulating the camera view of a drone controlled via MAVLink.
The generated video output is then processed using computer vision to detect and estimate the pose of AprilTag markers and implemented visual based precision landing.

The vision landing algorithm is published here:
https://github.com/kripper/vision-landing-2
