#!/usr/bin/env python  
import rospy
import math
"""
TODO add any missing imports (e.g. custom messages/services)
"""

# load private parameters
svc_ik = rospy.get_param("~inv_kin_service", "/do_ik")     

# DH Parameters for ROB3/TR5
d1 = 0.275
a2 = 0.200
a3 = 0.130
d5 = 0.130

"""
TODO Implement the callback to handle the inverse kinematics calculations 
"""
def doInverseKinematics(srv):
    return 

if __name__ == "__main__":
    rospy.init_node('tr5_inverse_kinematics')
    """
    TODO Create and advertise the service for inverse kinematics
    """

    while not rospy.is_shutdown():
        rospy.spin()
    