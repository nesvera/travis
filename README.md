# travis

Tested on:

    Ubuntu 16.04 LTS
    ROS Kinetic

How to install:

    sudo apt-get install ros-kinetic-ackermann-*
    
    sudo apt-get install ros-kinetic-joy

    git clone https://github.com/dmalyuta/apriltags2_ros.git

    catkin_make install

How to use

    roslaunch autocone_gazebo empty_colin.launch

Travis on Jetson

    sudo apt-get install ros-kinetic-joy

    Install Microsoft xbox360 controller (http://kyubot.tistory.com/92)
    sudo apt-get install jstest-gtk joystick xboxdrv
    sudo xboxdrv --silent
    Create a file: /etc/rc.local
    File contents:
    #!/bin/sh -e
    xboxdrv --silent
    exit 0
    Save the file and make it executable with this command:
    sudo chmod +x /etc/rc.local

    sudo apt-get install ros-kinetic-ackermann*