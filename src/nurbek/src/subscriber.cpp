#include "ros/ros.h"
#include "std_msgs/Int32.h"

void idCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Received NU ID digit: %d", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_nurbek");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("nurbek", 1000, idCallback);

    ros::spin();

    return 0;
}
