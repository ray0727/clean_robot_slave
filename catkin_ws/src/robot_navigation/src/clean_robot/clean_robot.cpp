#include<stdio.h>
#include<string>
#include<string.h>
#include<math.h>
#include "clean_robot.h"
#include "map_scan.h"
#include "Positioning.h"
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "self_clean.h"

class Clean_Robot{
	public:
		ros::Subscriber map_sub,slam_pose_sub,scandata_sub,masterscandata_sub,pump_sub,command_sub;
		ros::Publisher  point_pub,slave_point_pub;
		ros::NodeHandle node_;
		List_node *clean_map;

	public:
		//----system status
		int robot_status;//store robot status 
		enum robot_status_define{Initial=1,Normal_Mode=2,Map_Complete=4,Map_notComplete=8,Robot_Move=16,Robot_Scan=32,Search_Unknown=64,Outer_Command=128};		
		bool odom_init; //determind code init
		bool scan_init;
		bool masterscan_init;
		bool isrobotmove=false;
		//----robot info
		Point robot_pose;//robot's now position 
		Point old_pose;//the old position of robot
		Pointf pose; 

		//----map info
		Map_Scan *map_;		//get map data,and Print data[y*width+x]
		int *map_data;
		Positioning *position;

		//----lidar info----
		sensor_msgs::LaserScan *scan;
		sensor_msgs::LaserScan *masterscan;

		//-----self clean
		Self_Clean sc;
		Pointf region[2];

	Clean_Robot();
	~Clean_Robot();
	void mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); //
	void slamposecallback(const geometry_msgs::PoseStamped::ConstPtr& Pose);
	void scandatacallback(const sensor_msgs::LaserScan::ConstPtr& scandata);
	void masterscandatacallback(const sensor_msgs::LaserScan::ConstPtr& masterscandata);
	void pumpcallback(const std_msgs::String::ConstPtr& bumpdetect);
	void commandcallback(const std_msgs::String::ConstPtr& command);	
	bool check_getmap(){return !(map_->mapcomplete) && !isrobotmove;} //map not complete and robot not move,it can get map

};

Clean_Robot::Clean_Robot(){
	//----initial subscribe and publish
	map_sub=node_.subscribe("clean_robot_map",1000,&Clean_Robot::mapcallback,this);
	slam_pose_sub=node_.subscribe("slave/slam_out_pose",1000,&Clean_Robot::slamposecallback,this);
	scandata_sub=node_.subscribe("slave/scan",1000,&Clean_Robot::scandatacallback,this);
	masterscandata_sub=node_.subscribe("master/scan",1000,&Clean_Robot::masterscandatacallback,this);
	pump_sub=node_.subscribe("slave/bump_detect",1000,&Clean_Robot::pumpcallback,this);
	command_sub=node_.subscribe("master_command",1000,&Clean_Robot::commandcallback,this);
	slave_point_pub=node_.advertise<geometry_msgs::PoseStamped>("slave_robot_point",1000);
	point_pub=node_.advertise<geometry_msgs::PoseStamped>("slave/move_base_simple/goal",1000);
	//----robot initial state
	robot_pose=Point(0,0);
	old_pose=Point(0,0);
	map_=new Map_Scan[1];
	robot_status=Initial;

	//----system state
	odom_init=false;
	scan_init=false;
	masterscan_init=false;	
	isrobotmove=false;
	ROS_INFO("Clean_Robot Bulid");

	//----lidar info----
	scan=new sensor_msgs::LaserScan [1];
	masterscan=new sensor_msgs::LaserScan [1];
	position=new Positioning [1];
}

Clean_Robot::~Clean_Robot(){
	delete [] map_;
	delete [] map_data;
	delete [] scan;
	delete [] masterscan;
	delete [] position;
}

void Clean_Robot::mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	ROS_INFO("In clean_robot.cpp:mapcallback");
	int width=msg->info.width;

	if(!(map_->mapinit)){
		ROS_INFO("map initail");		
		//map_initial
		double resolution=msg->info.resolution;
		map_data=new int [width*width];
		for(int i=0;i<width*width;i++)
			map_data[i]=msg->data[i];
		map_->map_init(resolution,width,map_data);
		sc.init(width,map_->map,resolution);
	}else if(!check_getmap()){
		//map_->print_map(robot_pose);
	}else{
		for(int i=0;i<width*width;i++)
			map_data[i]=msg->data[i];
		map_[0].checkmap(map_data,robot_pose);
	}

}

void Clean_Robot::slamposecallback(const geometry_msgs::PoseStamped::ConstPtr& Pose)
{
	//ROS_INFO("slam_pose:%lf,%lf",Pose->pose.position.x,Pose->pose.position.y);
	float diffx,diffy;
	if(map_->mapinit && position->init){
		Pointf tfstom=position[0].transformto_masterodom(Pointf(Pose->pose.position.x,Pose->pose.position.y));
		if(tfstom.x>0){
			diffx=0.5;
		}else{
			diffx=-0.5;
		}
		if(tfstom.y>0){
			diffy=0.5;
		}else{
			diffy=-0.5;
		}	
		robot_pose.x=(int)((tfstom.x)/map_->map_resolution+diffx);
		robot_pose.y=(int)(tfstom.y/map_->map_resolution+diffy);
		odom_init=true;
		ROS_INFO("robot pose:%d,%d",robot_pose.x,robot_pose.y);
		geometry_msgs::PoseStamped p;
		p.header.frame_id="map";
		p.header.stamp=ros::Time::now();
		p.pose.position.x=tfstom.x;
		p.pose.orientation=Pose->pose.orientation;
		p.pose.orientation.w=cos(acos(p.pose.orientation.w)+position->rotatedegree/2);
		p.pose.orientation.z=sin(asin(p.pose.orientation.z)+position->rotatedegree/2);
		slave_point_pub.publish(p);
		//point_pub.publish(p);
	}
}

void Clean_Robot::scandatacallback(const sensor_msgs::LaserScan::ConstPtr& scandata){
	int count=int((scandata->angle_max-scandata->angle_min)/scandata->angle_increment+1);
	//ROS_INFO("in scan data %d",count);
	if(!scan_init && count!=0){
		scan->angle_min=scandata->angle_min;
		scan->angle_max=scandata->angle_max;
		scan->angle_increment=scandata->angle_increment;
		scan->time_increment=scandata->time_increment;
		scan->scan_time=scandata->scan_time;
		scan->range_min=scandata->range_min;
		scan->range_max=scandata->range_max;
		std::vector<float> &data=scan->ranges;
		data.resize(count);
		for(int i=0;i<count;i++)
			data[i]=scandata->ranges[i];
		scan_init=true;
	}else{
		for(int i=0;i<count;i++)
			scan->ranges[i]=scandata->ranges[i];		
	}
}

void Clean_Robot::masterscandatacallback(const sensor_msgs::LaserScan::ConstPtr& scandata){

	//ROS_INFO("in master scan data");
	int count=(int)((scandata->angle_max-scandata->angle_min)/scandata->angle_increment+1);
	if(!masterscan_init && count!=0){
		masterscan->angle_min=scandata->angle_min;
		masterscan->angle_max=scandata->angle_max;
		masterscan->angle_increment=scandata->angle_increment;
		masterscan->time_increment=scandata->time_increment;
		masterscan->scan_time=scandata->scan_time;
		masterscan->range_min=scandata->range_min;
		masterscan->range_max=scandata->range_max;	
		std::vector<float> &data=masterscan->ranges;
		data.resize(count);
		for(int i=0;i<count;i++)
			data[i]=scandata->ranges[i];
		//position[0].calculatediff(masterscan,scan);
		masterscan_init=true;
	}else{
		for(int i=0;i<count;i++)
			masterscan->ranges[i]=scandata->ranges[i];

	}	
}

void Clean_Robot::pumpcallback(const std_msgs::String::ConstPtr& bumpdetect){
	if(robot_status!=Initial)
		return;
	ROS_INFO("%s",bumpdetect->data.c_str());
	if(!(scan_init && masterscan_init))
		return;
	if(strcmp(bumpdetect->data.c_str(),"HIGH")==0)
		position[0].calculatediff(masterscan,scan);
}

void Clean_Robot::commandcallback(const std_msgs::String::ConstPtr& command){
	ROS_INFO("region callback");
	if(robot_status & Initial)
		return;
	robot_status^=Outer_Command;
	std::string s=command->data;
	char * cstr = new char [s.length()+1];
  	std::strcpy (cstr, s.c_str());

  	// cstr now contains a c-string copy of str

  	char * p = std::strtok (cstr,"\n");
	p = std::strtok (NULL,"\n");
	int i=0;
  	for(i=0;i<2 && p!=0;i++)
  	{
		this->region[i].x=atof(p);
    	p = std::strtok(NULL,"\n");
		this->region[i].y=atof(p);
    	p = std::strtok(NULL,"\n");
  	}
	if(this->region[0].x<this->region[1].x){
		float t=this->region[0].x;
		this->region[0].x=this->region[1].x;
		this->region[1].x=t;
	}
	if(this->region[0].y<this->region[1].y){
		float t=this->region[0].y;
		this->region[0].y=this->region[1].y;
		this->region[1].y=t;
	}

  	delete[] cstr;
}

int main(int argc,char *argv[]){
	
	ros::init(argc,argv,"cleanrobot_node");
	Clean_Robot main;
	
	while(ros::ok()){
		ros::spinOnce();
		//ROS_INFO("main");
		switch(main.robot_status){
			case main.Initial:
				if(main.scan_init && main.masterscan_init){
					ROS_INFO("scan_initial");
					main.position[0].calculatediff(main.masterscan,main.scan);
					main.sc.set_tf(main.position[0].diffx,main.position[0].diffy,main.position[0].rotatedegree);
				}
				main.robot_status=main.Normal_Mode;
				break;
			case main.Normal_Mode:

				break;
			case (main.Normal_Mode | main.Outer_Command):
				main.sc.start_clean(main.region);
				main.robot_status=main.Robot_Move;
				if(!main.sc.send_point(main.point_pub))
					main.robot_status=main.Normal_Mode;
				break;
			case main.Robot_Move:
				if(main.sc.is_arrive(main.robot_pose)){
					if(main.sc.send_point(main.point_pub)){
						main.robot_status=main.Robot_Move;
					}else{
						main.robot_status=main.Normal_Mode;
					}
				}
				break;
		}
	}

	return 0;
}



