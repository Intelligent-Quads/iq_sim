#include <ros/ros.h>
#include <mavros_msgs/StatusText.h>
#include <sensor_msgs/Range.h>

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "sim_range_fider");
	ros::NodeHandle sim_range_fider("~");
    ros::Publisher range_pub = sim_range_fider.advertise<sensor_msgs::Range>("/mavros/setpoint_position/local", 10);
    sensor_msgs::Range lidar_msg;
    lidar_msg.min_range = .3;
    lidar_msg.max_range = 40;
    lidar_msg.range = 5;
    while(ros::ok())
    {
        lidar_msg.header.stamp = ros::Time::now();
        lidar_msg.header.frame_id = "downward_lidar";
        range_pub.publish(lidar_msg);
    }
}