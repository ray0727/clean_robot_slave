#include "clean_robot.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


class Positioning{
	
	
	public:
		//Point getmasterpoint(sensor_msgs::LaserScan ,int mapwidth,Point robot_pose);
		//Point 
		Point getmasterpoint(Point master);
		Pointf calculatediff(sensor_msgs::LaserScan *master,sensor_msgs::LaserScan *self);
		double calSD(std::vector<double> list,double total);
		Positioning();
		~Positioning();
		//void scancallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	
	
	public:
		Point master;
		double diffx,diffy;
		double rotatedegree;
		//ros::NodeHandle node_;
	
	
};

