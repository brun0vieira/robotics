#!/usr/bin/env python

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import std_msgs.msg

'''
    Receives a frame name in a topic /new_frame (std_msgs/String)
    and publishes a transform between /base_footprint and this new  
    frame with following values (x:1 y:1 z:1 roll:0 pitch:0 yaw:0)

    rostopic pub -r rate /new_frame std_msgs/String string
'''
def addFrame(msg):
    br.sendTransform((1,1,1),
                     tf.transformations.quaternion_from_euler(0,0,0),
                     rospy.Time.now(),
                     msg.data,
                     '/base_footprint')

if __name__ == '__main__':
    rospy.init_node('uav_tf_listener')
    listener = tf.TransformListener()
    
    br = tf.TransformBroadcaster()
    rospy.Subscriber('/new_frame', std_msgs.msg.String, addFrame)   
    
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            (trans_newframe,rot_newframe) = listener.lookupTransform('/base_footprint', '/nova_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        print('/base_footprint relative to the /map      \tx: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f' %(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]))
        print('/new_frame relative to the /base_footprint\tx: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f\n' %(trans_newframe[0], trans_newframe[1], trans_newframe[2], rot_newframe[0], rot[1], rot[2]))
        rate.sleep()