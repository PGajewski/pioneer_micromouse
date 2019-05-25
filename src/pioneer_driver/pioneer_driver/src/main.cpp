/*
 * @file    main.cpp
 * @authors Daniel Koguciuk <daniel.koguciuk@gmail.com>
 * @date    10.11.2015
 */

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_cmd_vel;

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    //VAR
    geometry_msgs::Twist msg;
    float linear = 0;
    float angular = 0;

    //JOY
    //if ((float)joy->axes.at(5)<-0.2 && (float)joy->axes.at(5)>-0.99*)     //Deadman switch
    {
        if (joy->axes.at(4)<-0.2 || joy->axes.at(4)>0.2)                  //Dead zone for driving forward
            linear = (float)joy->axes.at(4);

        if (joy->axes.at(3)<-0.2 || joy->axes.at(3)>0.2)                  //Dead zone for touring around
            angular = (float)joy->axes.at(3);
    }

    //MSG
    msg.linear.x = linear;
    msg.angular.z = angular;
    pub_cmd_vel.publish(msg);
}

int main(int argc, char** argv)
{
    //Init ros node
    ros::init(argc, argv, "manual");

    //Ros variables
    ros::NodeHandle n("~");
    ros::Rate rate(10);

    //Robot name
    std::string ns = n.getNamespace();
    std::string parent_ns = ros::names::parentNamespace(ns);

    //Sub & pub
    ros::Subscriber sub_joy 	= n.subscribe<sensor_msgs::Joy>("/" + parent_ns + "/joy", 1, &joyCallback);
    pub_cmd_vel  = n.advertise<geometry_msgs::Twist>("/" + parent_ns + "/aria/cmd_vel",1);

    //Msg

    while(n.ok())
    {
        //Sleep
        rate.sleep();

        //Spin
        ros::spin();
    }

    return 0;
}

