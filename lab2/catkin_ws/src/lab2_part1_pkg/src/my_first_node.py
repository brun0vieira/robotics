#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node("hello_world_py_node")
    log_freq = rospy.get_param('/hello_publish_frequency', 1)
    rate = rospy.Rate(log_freq)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello World!")
        rate.sleep()

    rospy.loginfo("Node is shutting down...")

    