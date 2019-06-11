#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/console.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <chrono>
#include <fstream>

using namespace std;

/**
 * @brief robot_name            Current robot name.
 */
const string robot_name="robin";

/**
 * @brief sub_odom               Robot odometry subscriber.
 */
ros::Subscriber sub_odom;

/**
 * @brief sub_laser		Laser points cloud.
 */
ros::Subscriber sub_laser;

/**
 * @brief pub_vel               Robot velocity publisher.
 */
ros::Publisher pub_vel;

//Position and orientation variables.
std::atomic<double> pos_x{0};
std::atomic<double> pos_y{0};
std::atomic<double> pos_z{0};

std::atomic_bool is_left_wall{false};
std::atomic_bool is_front_wall{false};
std::atomic_bool is_right_wall{false};

std::atomic<double> roll{0};
std::atomic<double> pitch{0};
std::atomic<double> yaw{0};

sig_atomic_t volatile is_running = 0;

std::mutex m;
std::condition_variable cv;

const float  PI_F=3.14159265358979f;

std::string file;

//Functions.
void waitForNewPose() throw(std::exception)
{
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, []{return true;});
    if(!is_running)
	throw std::exception();
}

void move(float forward, float rotate){
    const float linear_vel_gain = 0.05;
    const float angular_vel_gain = 0.1;
    geometry_msgs::Twist vel;
    vel.linear.x = linear_vel_gain * forward;
    vel.angular.z = angular_vel_gain * rotate;
    pub_vel.publish(vel);
    //cout << "Published!" << endl;
}


void stop(){
    std::cout << "Stop engines" << std::endl;
    move(0,0);
}

void runForward() throw(std::exception) {
    std::cout << "Run forward" << std::endl;
    const float distance = 1;
    auto start_pos_x = pos_x.load();
    auto start_pos_y = pos_y.load();

    //Wait for end position.
    while(1)
    {
	waitForNewPose();
        if((pow(start_pos_x-pos_x.load(),2) + pow(start_pos_y-pos_y.load(),2)) >= pow(distance, 2))
	{
	    break;
	} else {
	    move(1,0);
	}
    }
    //Stop engines.
    stop();
    cout << "Forward end" << endl;
}

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

void turnLeft() throw(std::exception){
    auto actual_yaw = yaw.load();

    std::cout << "Start turn left." << std::endl;

    //Start turning
    //move(0,1);

    //Wait for end orientation based on start orientation.
    switch(getQuarter(actual_yaw))
    {
	case 1:
	case 3:
	case 4:
	{
	    auto end_yaw = actual_yaw + PI_F/2; // - 0.03;
	    //Wait for move ending.
	    while(1)
    	    {
        	waitForNewPose();
        	if(yaw.load() >= end_yaw)
        	{
            	    break;
        	} else {
		    move(0,1);
		}
    	    }
	    break;
	}
	case 2:
	{
	    auto end_yaw = -1.5 * PI_F + actual_yaw; // - 0.03;
	    //Wait for change yaw sign.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() <= 0)
                {
                    break;
                } else {
                    move(0,1);
                }

            }
	    //Wait for move ending.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() >= end_yaw)
                {
                    break;
                } else {
                    move(0,1);
                }

            }
	    break;
	}
    }
    //Stop engines.
    stop();
    std::cout << "Stop turn left." << std::endl;


}

void turnRight() throw(std::exception) {
    auto actual_yaw = yaw.load();

    std::cout << "Start turn right." << std::endl;

    //Start turning
    //move(0,-1);

    //Wait for end orientation based on start orientation.
    switch(getQuarter(actual_yaw))
    {
        case 1:
        case 2:
        case 4:
	{
            auto end_yaw = actual_yaw - PI_F/2; // + 0.02;
            //Wait for move ending
            while(1)
            {
                waitForNewPose();
                if(yaw.load() <= end_yaw)
                {
                    break;
                } else {
                    move(0,-1);
                }

            }
            break;
	}
        case 3:
	{
            auto end_yaw = 1.5 * PI_F - actual_yaw; // + 0.02;
            //Wait for change yaw sign.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() >= 0)
                {
                    break;
                } else {
                    move(0,-1);
                }

            }
            //Wait for move ending.
            while(1)
            {
                waitForNewPose();
                if(yaw.load() <= end_yaw)
                {
                    break;
                } else {
                    move(0,-1);
                }

            }
	    break;
	}
    }

    //Stop engines.
    stop();
    std::cout << "Stop turn right." << std::endl;

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    const int decision_threshold = 18;
    const int checks_number = 20;
    const float wall_distance = 0.5;
    const int left_range_min = 50;
    const int left_range_max = 150;
    const int front_range_min = 300;
    const int front_range_max = 400;
    const int right_range_min = 550;
    const int right_range_max = 650;

    //Check is wall near robot.
    //Left
    int hit_number = 0;
    for(int j=0; j < checks_number; ++j)
    {
	//Get a random point number.
	int pos = left_range_min + (rand()%(left_range_max - left_range_min));
	if(!isnan(scan->ranges[pos]) && scan->ranges[pos] < wall_distance)
	    ++hit_number;
    }

    if(hit_number < decision_threshold)
	is_left_wall = false;
    else
	is_left_wall = true;

    //Front
    hit_number = 0;
    for(int j=0; j < checks_number; ++j)
    {
        //Get a random point number.
        int pos = front_range_min + (rand()%(front_range_max - front_range_min));
        if(!isnan(scan->ranges[pos]) && scan->ranges[pos] < wall_distance)
            ++hit_number;
    }

    if(hit_number < decision_threshold)
        is_front_wall = false;
    else
        is_front_wall = true;

    //Right
    hit_number = 0;
    for(int j=0; j < checks_number; ++j)
    {
        //Get a random point number.
        int pos = right_range_min + (rand()%(right_range_max - right_range_min));
        if(!isnan(scan->ranges[pos]) && scan->ranges[pos] < wall_distance)
            ++hit_number;
    }

    if(hit_number < decision_threshold)
        is_right_wall = false;
    else
        is_right_wall = true;

    //std::cout << is_left_wall.load() << "|" << is_front_wall.load() << "|" << is_right_wall.load() << std::endl;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //Get odometry
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
    pos_z = msg->pose.pose.position.z;

    //Convert from quatermions for
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double temp_roll, temp_pitch, temp_yaw;
    m.getRPY(temp_roll, temp_pitch, temp_yaw);
//    cout << "Roll: " << temp_roll << " Pitch: " << temp_pitch << " Yaw: " << temp_yaw << endl;
    roll = temp_roll;
    pitch = temp_pitch;
    yaw = temp_yaw;

    cv.notify_all();
}

void build_map(const string file, int max_height, int max_width) throw(std::runtime_error)
{
    //Create map matrix and initialize.
    int** map = new int*[max_height];
    for(int i = 0; i < max_height; ++i)
    {
	map[i] = new int[max_width];
	for(int j=0; j<max_width; ++j)
	    map[i][j] = 0;
    }

    int height, width = 0;
    int global_orientation = 0;
    //Analize actual_position - try always turn right.
    while(1){
	int actual_value = 0;
	if(map[height][width] == 0)
	{
	//Build map.
	if(is_left_wall)
	    actual_value += 1;
	if(is_front_wall)
	    actual_value += 2;
	if(is_right_wall)
	    actual_value += 4;
	map[height][width] = actual_value;
	} else {
	actual_value = map[height][width];
	}
	//Drive.
	if(actual_value >= 4)
	{
	    turnRight();
	    runForward();
	    //Update orientation and position.
	    switch(global_orientation)
	    {
		case 0:
		    ++ width;
		    global_orientation = 1;
		    break;
		case 1:
		    --height;
		    global_orientation = 3;
		    break;
		case 2:
		    ++height;
		    global_orientation = 0;
		    break;
		case 3:
		    --width;
		    global_orientation = 2;
		    break;
	    }
	} else if(actual_value >=2){
	    runForward();
            switch(global_orientation)
            {
                case 0:
                    ++height;
                    break;
                case 1:
                    ++width;
                    break;
                case 2:
                    --width;
                    break;
                case 3:
                    --height;
		    break;
	    }
	} else if(actual_value == 1){
	    turnLeft();
	    runForward();
            switch(global_orientation)
            {
                case 0:
                    --width;
                    global_orientation = 2;
                    break;
                case 1:  
                    ++height;
                    global_orientation = 3;
                    break;
                case 2:
                    ++height;
                    global_orientation = 0;
                    break;  
                case 3:
                    --width;
                    global_orientation = 2;
		    break;
	    }
	} else {
	//Turn back.
	    turnLeft();
	    turnLeft();
	    runForward();
	}
    //Stop condition.
    if(width == 0 && height == 0)
    {
	break;
    }
    }
}

std::list<char> get_commands_list(const string file) throw(std::runtime_error)
{
    ifstream commands_file(file.c_str());
    std::list<char> command_list;
    if(commands_file){
        char command;
        while(!commands_file.eof()){
            commands_file.get(command);
	    std::cout << command;
            command_list.push_back(command);
        }
	return command_list;
    }
    else{
        throw std::runtime_error("No such file.");
    }
}

void control_loop()
{
    try{
        std::this_thread::sleep_for (std::chrono::seconds(2));
	std::list<char> command_list = get_commands_list(file);
        //while(is_running)
	//{
	//    turnRight();
	//}
        {
            for (auto it = command_list.begin(); it != command_list.end(); ++it){
		//std::cout << *it << std::endl;
        	switch(*it){
		    case 's':
			runForward();
			break;
		    case 'R':
			turnRight();
			break;
		    case 'L':
			turnLeft();
			break;
		    default:
			throw std::runtime_error("Unrecognised command!");
			break;
		}
	    }
    	}
    } catch(std::exception e) {
        stop();
	std::cout << e.what();
    }
}

void build_map_loop()
{
    try{
        std::this_thread::sleep_for (std::chrono::seconds(2));
	build_map(file, 5, 5);
    } catch(std::exception e) {
        stop();
	std::cout << e.what();
    }
}

void signalHandler(int dummy)
{
    std::cout << "Signal" << std::endl;
    is_running = 0;
    cv.notify_all();
    ros::shutdown();
}

/**
 * @brief main                      Main routine.
 * @param argc                      Used as default in ros.
 * @param argv                      Used as default in ros.
 * @return
 */
int main(int argc, char** argv)
{
    struct sigaction act;
    act.sa_handler = signalHandler;
    sigaction(SIGINT, &act, NULL);

    //ROS node initialization
    ros::init(argc, argv, "pioneer_control", ros::init_options::NoSigintHandler);
    ros::NodeHandle n("~");
    ros::Rate rate(30);

    //Get params.
    std::string mode, sim;
    n.getParam("mode", mode);
    n.getParam("file", file);
    n.getParam("sim", sim);


    // Subscriber and publisher init
    if(sim.compare("sim"))
    {
	std::cout << "Start simulation" << std::endl;
        sub_odom = n.subscribe(string("/pioneer/odom"), 10, &odomCallback);
        pub_vel = n.advertise<geometry_msgs::Twist>(string("/pioneer/cmd_vel"), 10);
    } else {
	std::cout << "Start robot" << std::endl;
	sub_odom = n.subscribe(string("/") + robot_name + string("/aria/pose"), 10, &odomCallback);
    	sub_laser = n.subscribe<sensor_msgs::LaserScan>(string("/") + robot_name + string("/scan"), 10, &laserCallback);
        pub_vel = n.advertise<geometry_msgs::Twist>(string("/") + robot_name + string("/aria/cmd_vel"), 10);
    }

    //Init random number generator.
    srand (time(NULL));

    is_running = 1;

    if(mode.compare("build"))
    {
	thread build_thread(build_map_loop);
	while(n.ok())
	{
            //Spin
    	    ros::spinOnce();

            //Sleep
            rate.sleep();
	}
    	build_thread.join();
    } else if(mode.compare("drive"))
    {
    	thread control_thread(control_loop);

	while(n.ok())
	{
            //Spin
    	    ros::spinOnce();

            //Sleep
            rate.sleep();
	}
    	control_thread.join();
    }
    return 0;
}
