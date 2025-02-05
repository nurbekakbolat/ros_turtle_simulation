#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"
#include <cmath>

ros::Publisher pub;
turtlesim::Pose current_pose;
bool reached_target = false;
double target_x, target_y;

double corners[4][2] = {
    {1.0, 1.0},
    {9.0, 1.0}, 
    {9.0, 9.0}, 
    {1.0, 9.0}  
};
int corner_index = 0;

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    current_pose = *msg;

    double distance = sqrt(pow(target_x - current_pose.x, 2) + pow(target_y - current_pose.y, 2));

    if (distance < 0.1) {
        reached_target = true;
    }
}

void moveToNextCorner() {
    geometry_msgs::Twist move_cmd;

    target_x = corners[corner_index][0];
    target_y = corners[corner_index][1];
    reached_target = false;

    ros::Rate rate(10);
    while (!reached_target && ros::ok()) {
        double angle_to_target = atan2(target_y - current_pose.y, target_x - current_pose.x);

        if (fabs(current_pose.theta - angle_to_target) > 0.1) {
            move_cmd.linear.x = 0.0; 
            move_cmd.angular.z = (angle_to_target - current_pose.theta) > 0 ? 0.5 : -0.5;
        } else {
            move_cmd.linear.x = 2.0;
            move_cmd.angular.z = 0.0;
        }

        pub.publish(move_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    move_cmd.linear.x = 0.0;
    move_cmd.angular.z = 0.0;
    pub.publish(move_cmd);

    corner_index = (corner_index + 1) % 4;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_square_dynamic");
    ros::NodeHandle nh;

    ros::ServiceClient killClient = nh.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill killSrv;
    killSrv.request.name = "turtle1";
    killClient.call(killSrv);

    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawnSrv;
    spawnSrv.request.x = 5.5;
    spawnSrv.request.y = 5.5;
    spawnSrv.request.theta = 0.0; 
    spawnSrv.request.name = "Nurbek_Turtle";
    spawnClient.call(spawnSrv);

    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("/Nurbek_Turtle/teleport_absolute");
    turtlesim::TeleportAbsolute teleportSrv;
    teleportSrv.request.x = 1.0; 
    teleportSrv.request.y = 1.0;
    teleportSrv.request.theta = 0.0; 
    if (teleportClient.call(teleportSrv)) {
        ROS_INFO("Teleported turtle to bottom-left corner.");
    } else {
        ROS_ERROR("Failed to teleport the turtle.");
        return 1;
    }

    pub = nh.advertise<geometry_msgs::Twist>("/Nurbek_Turtle/cmd_vel", 1);

    ros::Subscriber sub = nh.subscribe("/Nurbek_Turtle/pose", 10, poseCallback);

    ros::Duration(1.0).sleep();

    while (ros::ok()) {
        moveToNextCorner();
        ros::spinOnce();
    }

    return 0;
}
