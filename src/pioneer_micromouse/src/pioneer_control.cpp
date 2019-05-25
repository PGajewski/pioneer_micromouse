#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <ros/console.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std;

/**
 * @brief robot_name            Current robot name.
 */
const string robot_name="robin";

/**
 * @brief pub_vel               Robot odometry subscriber.
 */
ros::Subscriber sub_odom;

/**
 * @brief pub_vel               Robot velocity publisher.
 */
ros::Publisher pub_vel;

//Position and orientation variables.
std::atomic<double> pos_x{0};
std::atomic<double> pos_y{0};
std::atomic<double> pos_z{0};

std::atomic<double> roll{0};
std::atomic<double> pitch{0};
std::atomic<double> yaw{0};

/**
 * @brief m                     Mutex for synchronizing coordinates.
 */
std::mutex m;

/**
 * @brief cv                    Condition variable to waiting and notifying about pose changing.
 */
std::condition_variable cv;
const float  PI_F=3.14159265358979f;

//Functions.

/**
 * @brief waitForPose            Function synchronizing pose changing.
 */
void waitForNewPose()
{
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, []{return true;});
}

/**
 * @brief move                    Move robot.
 * @param forward                 Velocity on forward.
 * @param rotate                  Velocity in rotating.
 */
void move(float forward, float rotate){
    const float linear_vel_gain = 1;
    const float angular_vel_gain = 2;
    geometry_msgs::Twist vel;
    vel.linear.x = linear_vel_gain * forward;
    vel.angular.z = angular_vel_gain * rotate;
    pub_vel.publish(vel);
    cout << "Published!" << endl;
}

/**
 * @brief stop                     Stop engines.
 */
void stop(){
    move(0,0);
}

/**
 * @brief runForward                Run robot forward on constant distance routine.
 */
void runForward(){
    cout << "Ender" << endl;
    const float distance_pow = 1;
    auto start_pos_x = pos_x.load();
    auto start_pos_y = pos_y.load();

    //Start moving.
    move(1,0);

    //Wait for end position.
    while(1)
    {
        waitForNewPose();
        if((pow(start_pos_x-pos_x.load(),2) + pow(start_pos_y-pos_y.load(),2)) >= distance_pow)
        {
            break;
        }
    }
    //Stop engines.
    stop();
}

/**
 * @brief getQuarter            Get quarter of actual pose.
 * @param angle                 Angle in coordinates system.
 * @return quarter              Actual pose quarter.  
 */
unsigned int getQuarter(float angle)
{
    if( angle > PI_F/2)
        return 2;
    else if( angle < PI_F/2 && angle >= 0)
        return 1;
    else if( angle < 0 && angle >= -PI_F/2)
        return 4;
    else
        return 3;
}

/**
 * @brief turnLeft              Turn robot left by 90 degrees.
 */
void turnLeft(){
    auto actual_yaw = yaw.load();

    //Start turning
    move(0,1);

    //Wait for end orientation based on start orientation.
    switch(getQuarter(actual_yaw))
    {
	    case 1:
        case 3:
        case 4:
        {
            auto end_yaw = actual_yaw + PI_F/2;
            //Wait for move ending.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() >= end_yaw)
                {
                    break;
                }
            }
            break;
        }
        case 2:
        {
            auto end_yaw = -1.5 * PI_F + actual_yaw;
            //Wait for change yaw sign.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() <= 0)
                {
                    break;
                }
            }
            //Wait for move ending.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() >= end_yaw)
                {
                    break;
                }
            }
            break;
        }
    }
    //Stop engines.
    stop();
}

/**
 * @brief turnRight              Turn robot right by 90 degrees.
 */
void turnRight(){
    auto actual_yaw = yaw.load();

    //Start turning
    move(0,-1);

    //Wait for end orientation based on start orientation.
    switch(getQuarter(actual_yaw))
    {
        case 1:
        case 2:
        case 4:
        {
            auto end_yaw = actual_yaw - PI_F/2;
            //Wait for move ending.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() <= end_yaw)
                {
                    break;
                }
            }
            break;
        }
        case 3:
	    {
            auto end_yaw = 1.5 * PI_F - actual_yaw;
            //Wait for change yaw sign.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() >= 0)
                {
                    break;
                }
            }

            //Wait for move ending.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() > end_yaw)
                {
                    break;
                }
            }
	        break;
	    }
    }

    //Stop engines.
    stop();
}

/**
 * @brief odomCallback          Robot odometry change callback.
 * @param msg                   Pose ROS message.
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //Get odometry
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
    pos_z = msg->pose.pose.position.z;

    //Convert from quatermions for rpy
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double temp_roll, temp_pitch, temp_yaw;
    m.getRPY(temp_roll, temp_pitch, temp_yaw);
    cout << "Roll: " << temp_roll << " Pitch: " << temp_pitch << " Yaw: " << temp_yaw << endl;
    roll = temp_roll;
    pitch = temp_pitch;
    yaw = temp_yaw;

    //Notify new pose.
    cv.notify_all();
}

/**
 * @brief controlLoop               Main control loop.
 */
void controlLoop()
{
    ros::NodeHandle n("~");
    try{

        while(n.ok())
        {
            //TODO: Main algorithm.
            runForward();
        }
    } catch(std::exception e) {
        stop();
        cerr << e.what();
    }
}

/**
 * @brief main                      Main routine.
 * @param argc                      Used as default in ros.
 * @param argv                      Used as default in ros.
 * @return
 */
int main(int argc, char** argv)
{
    //ROS node initialization
    ros::init(argc, argv, "pioneer_control");
    ros::NodeHandle n("~");
    ros::Rate rate(30);

    // Subscriber and publisher init
    sub_odom = n.subscribe(string("/") + robot_name + string("/aria/pose"), 10, &odomCallback);
    pub_vel = n.advertise<geometry_msgs::Twist>(string("/") + robot_name + string("/aria/cmd_vel"), 10);

    thread control_thread(controlLoop);

	while(n.ok())
	{
            //Spin
    	    ros::spinOnce();

            //Sleep
            rate.sleep();
	}
    control_thread.join();

    return 0;
}
