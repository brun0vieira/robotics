#!/usr/bin/env python
import rospy
from lab2_communication.msg import robot_id

def handle_robotid_msg_callback(msg):
    rospy.loginfo('Received message from Robot %d, model %s...' % (msg.id, msg.model))

if __name__ == '__main__':
    rospy.init_node('lab2_comm_subscriber_node', anonymous=True)
    rospy.Subscriber('lab2_robot_id_topic', robot_id, handle_robotid_msg_callback)
    rospy.spin()