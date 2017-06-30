#!/usr/bin/env python

import sys
import rospy
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController

if __name__ == "__main__":
    rospy.init_node('stop_mfj4_controller', anonymous=True)
    running = False

    while not running:
        rospy.wait_for_service('controller_manager/list_controllers', 20.0)
        list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
        running_controllers = list_controllers()
        for controller_state in running_controllers.controller:
            if controller_state.name == 'sh_rh_mfj4_position_controller' and controller_state.state == 'running':
                rospy.wait_for_service('controller_manager/switch_controller', 20.0)
                switch_controllers = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
                switched_controllers = switch_controllers(None, ['sh_rh_mfj4_position_controller'],
                                                      SwitchController._request_class.BEST_EFFORT)
                if switched_controllers.ok:
                    running = True
                    rospy.logerr("stopped controller for mfj4")
