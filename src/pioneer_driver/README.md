# About pioneer_lowlevel

This is bridge package between ROS framework and hardware layer of the pionner robots. 

# Pioneer hardware

![Schemat.png](https://bitbucket.org/repo/o4okj9/images/120914354-Schemat.png)

The brain of both pioneer robots is Jetson TK1 developer board. The detail hardware scheme is shown on the picture above - main features:

* Jetson is powered with 12V output from pioneer power board and it could take up to 4 Amps.
* Jetson's CPU is working with 1.8V so serial communication with any other hardware powered with higher voltage need additional voltage shift board (as listed down below). Currently there is only one voltage shift board for communication with IMU 3.3V.
* Jetson has 2 USB's placed on two different busses. It is very important from the perspective of connecting two high bandwidth devices (like cameras). At this moment there is a USB HUB connected to microUSB connector with Xbox joy. On the second USB a Hokuyo laser scanner is connected, but Asus Xtion camera is planned in the future.
* Laser scanner and Xbox joy are an optional equipment (currently available only in batman robot).
* **VERY IMPORTANT:** Internal RS232 (SERIAL) port of pioneer robot is different from the external (HOST) one. HOST port uses only TX/RX signals but SERIAL requires logic 1 on DTR signal (pin 4) with positive voltage level (about 5V). To fill this requirement you have to loop DTR (pin 4) with RI signal (pin 9) on the robot side.

Detailed information about used hardware:

* [Jetson TK1 Development Board](http://www.nvidia.com/object/jetson-tk1-embedded-dev-kit.html)
* [Razor 9DoF IMU](http://botland.com.pl/czujniki-9dof-imu/2949-razor-imu-akcelerometr-zyroskop-i-magnetometr-9-stopni-swobody-sparkfun.html)
* [Voltage level shiter](http://botland.com.pl/rozszerzenia-gpio-nakladki-hat-do-raspberry-pi-2/2259-konwerter-poziomow-logicznych-dwukierunkowy-czterokanalowy-sparkfun.html)
* [Xbox wireless controller](http://www.microsoft.com/hardware/pl-pl/p/xbox-360-wireless-controller-for-windows/JR9-00006#overview)
* [Hokuyo URG-04LX-UG01 laser scanner](http://www.hokuyo.pl/index.php?site=products&type=1169&details=9411)

# File permissions

There are several devices used in this package and each of them is used via standard linux file mechanism:

* /dev/ttyS0     (pioneer robot)
* /dev/ttyTHS1   (Razor IMU)
* /dev/ttyAMC0   (Hokuyo scanner)
* /dev/input/js0 (Xbox joystick)

The standard jetson user needs read/write permissions to access all those files. Dialout group as a default have permissions to read and write **ttyTHS1** and **ttyAMC0** files so you don't need to do anything, because default ubuntu user belongs to this group. Dialout don't have read/write permissions to **js0** file, so just type the following command (only once per every system flash):

    $ sudo chown -R root:dialout /dev/input

Things get a little bit more complicated with **ttyS0** because it is configured to be used by getty as a linux service starting every system boot automatically. In the newer versions of ubuntu (since 10.04) the upstart mechanism is used to manage those services. Every file matching the following template: **/etc/init/*.conf** is treated as service, so you can find **/etc/init/ttyS0.conf** file configuring the **ttyS0** as a linux standard console. You can disable this mechanism with (once per every system flash):

    $ sudo sh -c 'echo manual >> /etc/init/ttyS0.override'

Next boot ttyS0 service will be disabled. One can still start it with standard upstart mechanism.

# Set up

First of all you will need to set up ROS framework on Jetson platform - you can follow [this instructions](http://wiki.ros.org/indigo/Installation/UbuntuARM). After installation you can paste this commands to your .bashrc file:

    ##### CUDA #####
    export PATH=/usr/local/cuda-6.5/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib:$LD_LIBRARY_PATH
    export __GL_PERFMON_MODE=1

    ##### ROS #####
    source /opt/ros/indigo/setup.bash
    unset GTK_IM_MODULE

    WORKSPACE_BASH=/home/ubuntu/pioneer_ws/devel/setup.bash
    if [ -e $WORKSPACE_BASH ]
    then
        source $WORKSPACE_BASH
    fi

    MY_ROS_IP=192.168.1.130
    export ROS_IP=$MY_ROS_IP
    export ROS_HOSTNAME=$MY_ROS_IP
    export ROS_MASTER_URI=http://$MY_ROS_IP:11311

They set up all needed environment variables to use both CUDA and ROS system. Remember to set WORKSPACE_BASH and MY_ROS_IP variables! Then you can type following statement in terminal to get the all default packages needed for proper work of this package:

    $ sudo apt-get install ros-indigo-joy ros-indigo-hokuyo-node libaria2 libaria-dev ros-indigo-xacro ros-indigo-robot-state-publisher ros-indigo-tf

Now you will need rosaria package ([instruction here](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)):

    $ nawigate to your ros workspace / src
    $ git clone https://user_name@bitbucket.org/rrgwut/pioneer_driver.git
    $ git clone https://github.com/amor-ros-pkg/rosaria.git
    $ git clone https://github.com/KristofRobot/razor_imu_9dof.git
    $ cd ..
    $ catkin_make

Where user_name is your bitbucket user name. 

# Usage

There is one flexible lunchfile with some required args specifying what components do you want to run. Example usage for **batman** robot:

    $ roslaunch pioneer_driver pioneer_driver.launch robot_name:=batman aria:=true urdf:=true imu:=true scanner:=true joy:=true joy_control:=false --screen

And for **robin** robot:

    $ roslaunch pioneer_driver pioneer_driver.launch robot_name:=robin aria:=true urdf:=true imu:=true scanner:=false joy:=true joy_control:=false --screen

Required arguments:

* robot_name  - specifies name of the robot (batman or robin)
* aria        - specifies if aria should be lanuched
* imu         - specifies if imu node should be launched
* urdf        - specifies if urdf model should be lanuched
* scanner     - specifies if hokuyo laser scanner should be lanuched
* joy         - specifies if xbox controller node should be lanuched
* joy_control - specifies if control of the robot via joy should be lanuched

Good luck and have fun!