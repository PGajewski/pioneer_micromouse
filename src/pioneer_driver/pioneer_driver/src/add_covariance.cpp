/*
 * @file    main.cpp
 * @authors Daniel Koguciuk <daniel.koguciuk@gmail.com>
 * @date    10.11.2015
 */

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/assign.hpp>

ros::Publisher pub_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr &joy)
{
    //VAR
    nav_msgs::Odometry msg;

    //Copy!
    msg.header = joy->header;
    msg.child_frame_id = joy->child_frame_id;
    msg.pose = joy->pose;
    msg.twist = joy->twist;

    //Add cov to pose
    msg.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                 (0) (1e-3)  (0)  (0)  (0)  (0)
                                                 (0)   (0)  (1e-6) (0)  (0)  (0)
                                                 (0)   (0)   (0) (1e-6) (0)  (0)
                                                 (0)   (0)   (0)  (0) (1e-6) (0)
                                                 (0)   (0)   (0)  (0)  (0)  (1e-3) ;

    //Add cov to twist
    msg.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                  (0) (1e-3)  (0)  (0)  (0)  (0)
                                                  (0)   (0)  (1e-6) (0)  (0)  (0)
                                                  (0)   (0)   (0) (1e-6) (0)  (0)
                                                  (0)   (0)   (0)  (0) (1e-6) (0)
                                                  (0)   (0)   (0)  (0)  (0)  (1e-3) ;

    pub_odom.publish(msg);
}

int main(int argc, char** argv)
{
    //Init ros node
    ros::init(argc, argv, "add_cov_to_odom");

    //Ros variables
    ros::NodeHandle n("~");
    ros::Rate rate(10);

    //Robot name
    std::string ns = n.getNamespace();
    std::string parent_ns = ros::names::parentNamespace(ns);

    //Sub & pub
    ros::Subscriber sub 	= n.subscribe<nav_msgs::Odometry>("/" + parent_ns + "/aria/pose", 1, &odomCallback);
    pub_odom  = n.advertise<nav_msgs::Odometry>("/" + parent_ns + "/aria/pose_with_cov",1);

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
