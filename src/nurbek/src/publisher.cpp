#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher_nurbek");

    ros::NodeHandle n;

    ros::Publisher id_pub = n.advertise<std_msgs::Int32>("nurbek", 1000);

    std::vector<int> nu_id_digits = {2, 0, 1, 8, 9, 6, 1, 6, 4}; 

    ros::Rate loop_rate(1);

    size_t index = 0;
    while (ros::ok())
    {
        std_msgs::Int32 msg;

        msg.data = nu_id_digits[index];
        ROS_INFO("Publishing NU ID digit: %d", msg.data);

        id_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

        index = (index + 1) % nu_id_digits.size();

        if (index == 0)
        {
            ROS_INFO("Switching to 50 Hz");
            loop_rate = ros::Rate(50);
        }
    }

    return 0;
}
