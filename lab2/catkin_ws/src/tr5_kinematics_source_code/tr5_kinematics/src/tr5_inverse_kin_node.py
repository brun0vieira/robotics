#!/usr/bin/env python  
import rospy
import math

import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tr5_kinematics.srv import DoInverseKinematics, DoInverseKinematicsResponse

# load private parameters
svc_ik = rospy.get_param("~inv_kin_service", "/do_ik")     

# DH Parameters for ROB3/TR5
d1 = 0.275
a2 = 0.200
a3 = 0.130
d5 = 0.130

# receives a pose and returns the matrix entries
def poseToMatrix(pose):
    position = []
    orientation = []
    
    # translation expressed as a tuple (x,y,z)
    position.append(pose.position.x)
    position.append(pose.position.y)
    position.append(pose.position.z)

    # rotation quaternion expressed as a tuple (x,y,z,w)
    orientation.append(pose.orientation.x)
    orientation.append(pose.orientation.y)
    orientation.append(pose.orientation.z)
    orientation.append(pose.orientation.w)

    # converts a tf into a 4x4 matrix
    # |nx sx ax px|
    # |ny sy ay py|
    # |nz sz az pz|
    # | 0  0  0  1|
    m = tf.TransformerROS().fromTranslationRotation(position, orientation)

    nx = m[0][0]
    ny = m[1][0]
    nz = m[2][0]
    sx = m[0][1]
    sy = m[1][1]
    sz = m[2][1]
    ax = m[0][2]
    ay = m[1][2]
    az = m[2][2]
    px = m[0][3]
    py = m[1][3]
    pz = m[2][3]

    return (nx,ny,nz,sx,sy,sz,ax,ay,az,px,py,pz)

def arctg2(y,x):
    return 0 if y==0 and x==0 else math.atan2(y,x)

def sqrt(x):
    return 0 if x < 0 else math.sqrt(x)

# callback to handle the inverse kinematics calculations
def doInverseKinematics(srv):
    response = JointState()

    # converts req_pose into a matrix and get its entries
    (nx,ny,nz,sx,sy,sz,ax,ay,az,px,py,pz) = poseToMatrix(srv.req_pose)

    theta1 = arctg2(py,px) 
    theta234 = arctg2(az, ax*math.cos(theta1) + ay*math.sin(theta1)) 
    c_theta3 = (((px*math.cos(theta1) + py*math.sin(theta1) - d5*math.cos(theta234))**2) + ((pz-d1-d5*math.sin(theta234))**2) - a2**2 - a3**2) / (2*a2*a3)
    s_theta3 = sqrt(1-c_theta3**2)
    theta3 = math.atan(sqrt(1/(c_theta3**2)-1))
    theta2 = arctg2(((a2 + a3*math.cos(theta3))*(-d1+pz-d5*math.sin(theta234))), 
                    ((a2+a3*math.cos(theta3))*(-d5*math.cos(theta234) + math.cos(theta1)*px + math.sin(theta1)*py) + a3*math.sin(theta3)*(-d1+pz-d5*math.sin(theta234))))
    theta4 = theta2 - theta3
    theta5 = math.atan((-ny*math.cos(theta1) + nx*math.sin(theta1)) / (-sy*math.cos(theta1) + sx*math.sin(theta1)))

    response.header.stamp = rospy.Time.now()
    response.name = ['tr5shoulder_pan_joint', 'tr5shoulder_lift_joint', 'tr5elbow_joint', 'tr5wrist_1_joint', 'tr5wrist_2_joint', 'tr5left_finger_joint', 'tr5right_finger_joint']
    response.position = [theta1, theta2, theta3, theta4, theta5, 0.0, 0.0]
    response.effort = []
    response.velocity = []

    return DoInverseKinematicsResponse(response)

if __name__ == "__main__":
    rospy.init_node('tr5_inverse_kinematics')
    # create and advertise the service for inverse kinematics
    do_ik = rospy.Service(svc_ik, DoInverseKinematics, doInverseKinematics)

    while not rospy.is_shutdown():
        rospy.spin()
    