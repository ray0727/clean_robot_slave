#include "clean_robot.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


class Positioning{
	
	
	public:
		//Point getmasterpoint(sensor_msgs::LaserScan ,int mapwidth,Point robot_pose);
		//Point 
		Pointf transformto_masterodom(Pointf slave_pose);
		Pointf calculatediff(sensor_msgs::LaserScan *master,sensor_msgs::LaserScan *self);
		double calSD(std::vector<double> list,double total);
		double lidar_err;
		Positioning();
		~Positioning();
		//void scancallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	
	
	public:
		Point master;
		double diffx,diffy;
		double rotatedegree;
		bool init;
		//ros::NodeHandle node_;
	
	
};

