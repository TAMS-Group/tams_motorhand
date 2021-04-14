# tams_motorhand

This package provides different launch files as well es the robot description to bring up our Shadow motor hand.

### Installation

To use these launch files you will need the correct version of the shadow repositories, you will find a rosinstall file in the [rosinstalls](https://github.com/TAMS-Group/rosinstalls) package for indigo.

### Usage

The Shadow hand can be used in different setups, on the one side directly connected to the base station and on the other side together with the PR2.

#### ATTENTION

* To start the hand connected to the base station 
    - mannaully set ROS_MASTER_URI to localhost or run ``` local_ros ```.
    - need to bring up the second ethercat port manually. In /etc/netplan/02-tams-interfaces.yaml, enable "enp5s2" networt port then run ```sudo ip link set dev enp5s2 up```.
    - use the ```right_biotac_hand.launch``` file. The hand will be started in trajectory control mode. With the ```right_biotac_hand_no_controllers.launch``` the hand is started without controllers.


* To use the hand with the PR2 start ```right_biotac_hand_in_ns.launch``` on c2, this assumes tams_pr2_bringup's trixi.launch is already running.

* With ``` right_biotac_hand_gazebo.launch``` the simulation of the hand can be started as well as the moveit demo mode with ``` right_biotac_hand_moveit_demo.launch```