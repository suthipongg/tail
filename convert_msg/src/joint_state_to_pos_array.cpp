#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "iostream"

#define AMOUNT_MOTORS 4

std_msgs::Float64MultiArray joint_pos;

void joint_pos_callback(const sensor_msgs::JointState& joint)
{   
    joint_pos.data = joint.position;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_to_pos_arr");
    ros::NodeHandle n;

    ros::Publisher pub_joint_pos = n.advertise<std_msgs::Float64MultiArray>("/joint_positions", 1000);
    ros::Subscriber sub_join_state = n.subscribe("/joint_states", 1000, joint_pos_callback);

    ros::Rate rate(10);

    while (ros::ok())
    {
        pub_joint_pos.publish(joint_pos);
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();

    return 0;
}
