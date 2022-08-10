This project was modified from realsense-ros.
The original project name has been changed to keep the whole project  uniformed in naming;
	1.Change project name(replaced by raspbot_camera)
	2.Change msg and srv namespace
	3.Update the all namespace of  msg and srv  in folder 'src' and 'include' 
	4.Change the launch params
	5.Don't change the namespace of the wrapper and nodelet plugin

You can also install the offical package 
using apt command 'sudo apt install ros-{distribution}-realsense2-camera'.
It does not conflict with the package 'raspbot_camera' when existing both.

realsense-ros (Apache License 2.0) - https://github.com/IntelRealSense/realsense-ros
