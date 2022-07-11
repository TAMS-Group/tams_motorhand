# tams_motorhand

This package provides different launch files as well es the robot description to bring up our Shadow motor hand.

### Installation

To use these launch files you will need the correct version of the shadow repositories, you will find a rosinstall file in the [rosinstalls](https://github.com/TAMS-Group/rosinstalls) package for indigo.

### Usage

The Shadow hand can be used in different setups, on the one side directly connected to the base station and on the other side together with the PR2.

#### ATTENTION

* To start the hand connected to the base station 
    - recommend to use "shadowhand" username. If you use "pr2admin", mannaully set ROS_MASTER_URI to localhost or run ``` localros ```.
    - need to bring up the second ethercat port manually. In /etc/netplan/02-tams-interfaces.yaml, enable "eth1" networt port then run ```sudo ip link set dev eth1 up```.
    - use the ```right_biotac_hand_standalone.launch``` file. The hand will be started in trajectory control mode. With the ```right_biotac_hand_standalone_no_controllers.launch.launch``` the hand is started without controllers.


* To use the hand with the PR2 start ```right_biotac_hand_in_ns.launch``` on c2, this assumes tams_pr2_bringup's tams_pr2.launch is already running.

* With ``` right_biotac_hand_standalone_gazebo.launch``` the simulation of the hand can be started as well as the moveit demo mode with ``` right_biotac_hand_standalone_moveit_demo.launch```

* To limit the hand maximum force, copy this file:
  - left hand:
  ```
  cp tams_motor_hand/config/lh_motor_board_effort_controllers.yaml PATH_TO_sr_config/sr_ethercat_hand_config/controls/motors/lh/motor_board_effort_controllers.yaml
  ```
#### Left motor hand motor layout

![](tams_motorhand/media/left_shadow_hand_motor_layout.svg)
