#!/usr/bin/env python
import rospy
import sys
from lab2_communication.srv import Set_Robot_Model

if __name__ == '__main__':
    
    rospy.init_node('lab2_comm_client_node')

    if len(sys.argv) != 2:
        rospy.loginfo('usage: %s [model]' % str(sys.argv[0]))
        sys.exit(1)

    rospy.wait_for_service('Set_Robot_Model')

    srv_client_handler = rospy.ServiceProxy('Set_Robot_Model', Set_Robot_Model)

    try:
        resp = srv_client_handler.call(sys.argv[1])
        rospy.loginfo('New Robot ID Model: %d %s' % (resp.robotID.id, resp.robotID.model))
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s' % e)
    
