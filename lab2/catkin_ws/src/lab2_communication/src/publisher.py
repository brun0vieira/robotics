#!/usr/bin/env python
import rospy
from lab2_communication.msg import robot_id
from lab2_communication.srv import Set_Robot_Model, Set_Robot_ModelResponse

id = 100
model = 'irb120'

def set_model_service_callback(srv):
    global model
    rospy.loginfo('Set_Robot_Model call: %s -> %s' % (model,srv.model))
    model = srv.model
    res = robot_id()
    res.header.stamp = rospy.Time.now()
    res.header.frame_id = '/base_link'
    res.id = id
    res.model = model
    return Set_Robot_ModelResponse(res)

if __name__ == '__main__':
    rospy.init_node('lab2_comm_publisher_node', anonymous=True)

    pub = rospy.Publisher('lab2_robot_id_topic', robot_id, queue_size=10)

    srv_server_handler = rospy.Service('Set_Robot_Model', Set_Robot_Model, set_model_service_callback)

    rate = rospy.Rate(1)
    msg = robot_id()


    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.id = id
        msg.model = model
        pub.publish(msg)
        rospy.loginfo('Published robot_id message: %d %s' % (msg.id, msg.model))
        rate.sleep()