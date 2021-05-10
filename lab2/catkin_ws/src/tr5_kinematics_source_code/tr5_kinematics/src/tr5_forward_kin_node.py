#!/usr/bin/env python  
import rospy
import math

from sensor_msgs.msg import JointState as js
from geometry_msgs.msg import Pose as p
from tr5_kinematics_source_code.tr5_kinematics.srv import DoForwardKinematics

# load private parameters
svc_fk = rospy.get_param("~for_kin_service", "/do_fk") 

# DH Parameters for ROB3/TR5
d1 = 0.275
a2 = 0.200
a3 = 0.130
d5 = 0.130

# callback to handle the forward kinematics calculations
def doForwardKinematics(srv):
    shoulder_pan = srv.req_joint_state.position[0]
    shoulder_lift = srv.req_joint_state.position[1]
    elbow = srv.req_joint_state.position[2]
    wrist = srv.req_joint_state.position[3]
    
    c1 = math.cos(shoulder_pan)
    c2 = math.cos(shoulder_lift)
    c23 = math.cos(shoulder_lift + elbow)
    c234 = math.cos(shoulder_lift + elbow + wrist) 
    s1 = math.sin(shoulder_pan)
    s2 = math.sin(shoulder_lift)
    s23 = math.sin(shoulder_lift + elbow)
    s234 = math.sin(shoulder_lift + elbow + wrist)

    # c and s stands for cosine and sine, respectively
    # 1, 2, 3, 4 stands for the joints shoulder_pan, shoulder_lift, elbow and wrist, respectively
    response = p()
    response.position.x = d5*c1*s234 + a3*c1*c23 + a2*c1*c2
    response.position.y = d5*s1*s234 + a3*s1*c23 + a2*s1*c2
    response.position.z = -d5*c234 + a3*s23 + a2*s2 + d1

    return DoForwardKinematicsResponse(response)

if __name__ == "__main__":
    rospy.init_node('tr5_forward_kinematics')
    # create and advertise the service for forward kinematics
    fk_srv_handler = rospy.Service(svc_fk, DoForwardKinematics, doForwardKinematics)

    while not rospy.is_shutdown():
        rospy.spin()
    