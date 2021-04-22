#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hello_word_cpp_node");
    ros::NodeHandle nh;

    double pub_freq;
    nh.param("/hello_publish_frequency", pub_freq, 1.0);
    ros::Rate rate(pub_freq);

    while(ros::ok())
    {
        ROS_INFO("Hello World!");
        rate.sleep();
    }

    return 0;
}