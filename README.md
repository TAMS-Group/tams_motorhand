# tams_motorhand

This package provides different launch files as well es the robot description to bring up our Shadow motor hand.

### Installation

To use these launch files you will need the correct version of the shadow repositories, you will find a rosinstall file in the [rosinstalls](https://github.com/TAMS-Group/rosinstalls) package for indigo.

### Usage

The Shadow hand can be used in different setups, on the one side directly connected to the base station and on the other side together with the PR2.

#### ATTENTION
Currently the mfj4 joint is broken, so the controllers for this joint will not be loaded.

* To start the hand connected to the base station use the ```hand.launch``` file. The hand will be started in trajectory control mode. With the ```hand_no_controllers.launch``` the hand is started without controllers.

* To use the hand with the PR2 start ```hand_on_trixi.launch``` on c2, this assumes tams_pr2_bringup's trixi.launch is already running.

* With ```gazebo.launch``` the simulation of the hand can be started as well as the moveit demo mode with ```moveit_demo.launch```