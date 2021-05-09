#!/usr/bin/env python  
import rospy
import math
"""
TODO add any missing imports (e.g. custom messages/services)
"""

# load private parameters
svc_fk = rospy.get_param("~for_kin_service", "/do_fk") 

# DH Parameters for ROB3/TR5
d1 = 0.275
a2 = 0.200
a3 = 0.130
d5 = 0.130

"""
TODO Implement the callback to handle the forward kinematics calculations 
"""
def doForwardKinematics(srv):
    return

if __name__ == "__main__":
    rospy.init_node('tr5_forward_kinematics')
    """
    TODO Create and advertise the service for forward kinematics
    """

    while not rospy.is_shutdown():
        rospy.spin()
    