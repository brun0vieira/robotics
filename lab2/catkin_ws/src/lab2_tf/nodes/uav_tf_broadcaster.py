#!/usr/bin/env python
import roslib
import rospy
import tf
import std_msgs.msg

def handle_uav_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0,0, msg.data),
                     tf.transformations.quaternion_from_euler(0,0,0),
                     rospy.Time.now(),
                     'base_link',
                     'base_footprint')

if __name__ == '__main__':
    rospy.init_node('uav_tf_broadcaster')
    rospy.Subscriber('/uav/altitude', 
                     std_msgs.msg.Float32, 
                     handle_uav_pose) # The node subscribes to topic uav/altitude and runs handle_uav_pose on every incoming message
    rospy.spin()