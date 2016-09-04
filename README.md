CV Video
========

YamabROS is a library for integrating the [YP-Spur](https://openspur.org/redmine/projects) control library (used mainly by [Yamabico](http://www.roboken.iit.tsukuba.ac.jp/en/theme/) robots) to ROS.

Build & Install
---------------

YamabROS is built using [catkin](http://wiki.ros.org/catkin). Type the commands below on a terminal window to create a catkin workspace, clone the repository and build the sources:

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    $ git clone https://github.com/xperroni/yamabros.git
    $ cd ..
    $ catkin_make
