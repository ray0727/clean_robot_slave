#include "Positioning.h"
#include <math.h>
#include <vector>

Positioning::Positioning(){
    ROS_INFO("Position initial");
    this->lidar_err=0.03;
    this->init=false;
    this->rotatedegree=0;
    this->diffx=0;
    this->diffy=0;
}

Positioning::~Positioning(){

}

Pointf Positioning::calculatediff(sensor_msgs::LaserScan *master,sensor_msgs::LaserScan *self){
    int mcount=(int)((master->angle_max-master->angle_min)/master->angle_increment+1);
    int scount=(int)((self->angle_max-self->angle_min)/self->angle_increment+1);
    if(mcount==0 || scount==0)
        return Pointf(0,0);
    ROS_INFO("Positioning: mcount=%d",mcount);
    ROS_INFO("Positioning: scount=%d",scount);
    double mdegree=0,mdistance=0;
    double sdegree=0,sdistance=0;
    double degree;
    bool flag;
    std::vector<double> m;
    std::vector<double> s;
    //int m=0,s=0;
    for(int i=0;i<mcount;i++){   
        if(master->ranges[i]>=0.15 && master->ranges[i]<=0.30){            
            degree=master->angle_min+i*master->angle_increment;
            if(degree < -M_PI*0.9){
                flag=true;
            }
            if(flag && degree>M_PI*0.9){
                degree-=2*M_PI;
            }
            ROS_INFO("master degree,distance measure:%lf %lf",degree,master->ranges[i]);
            m.push_back(master->ranges[i]);
            mdistance+=master->ranges[i];
            mdegree+=degree;
        }
    }    
    mdegree/=(double)m.size();
    
    mdistance=calSD(m,mdistance)+this->lidar_err;
    flag=false;
    for(int i=0;i<scount;i++){
        if(self->ranges[i]>=0.15 && self->ranges[i]<=0.30){
            degree=self->angle_min+i*self->angle_increment;           
            if(degree < -M_PI*0.9){
                flag=true;
            }
            if(flag && degree>M_PI*0.9){
                degree-=2*M_PI;
            }
            ROS_INFO("slave degree,distance measure:%lf %lf",degree,self->ranges[i]);
            s.push_back(self->ranges[i]);
            sdistance+=self->ranges[i];
            sdegree+=degree;    
        }
    }
    ROS_INFO("test");
    sdegree/=(double)s.size();
    //sdegree-=M_PI;
    sdistance=calSD(s,sdistance)+this->lidar_err;
    ROS_INFO("Positioning: mdiffx,mdiffy(%lf,%lf)",mdistance*cos(mdegree),mdistance*sin(mdegree));//display robot center
    ROS_INFO("Positioning: sdiffx,sdiffy(%lf,%lf)",sdistance*cos(sdegree),sdistance*sin(sdegree));//display robot center
    diffx=(mdistance*cos(mdegree));
    diffy=mdistance*sin(mdegree);
    mdegree=atan2(mdistance*cos(mdegree),mdistance*sin(mdegree));//transform lidar locate to base link
    sdegree=atan2(sdistance*cos(sdegree),sdistance*sin(sdegree));//transform lidar locate to base link
    rotatedegree=mdegree-M_PI-sdegree;
    if(rotatedegree>M_PI){rotatedegree-=2*M_PI;}else if(rotatedegree<-M_PI){rotatedegree=+2*M_PI;}
    ROS_INFO("Positioning: diffx,diffy,rotatedegree(%lf,%lf,%lf)",diffx,diffy,rotatedegree);
    this->init=true;
    //return Pointf(cos(mdegree)-cos(sdegree)*,);
}

double Positioning::calSD(std::vector<double> list,double total){
    double rave=total/(double)list.size();
    double r_effective=0;
    double rSD=0;
    for(int i=0;i<list.size();i++){
        r_effective+=list[i]*list[i];
        rSD+=(list[i]-rave)*(list[i]-rave);
    }
    ROS_INFO("standard deviation= %lf",sqrt(rSD/(double)list.size()));
    return sqrt(r_effective/(double)list.size());
}

/*Point Positioning::getmasterpoint(sensor_msgs::LaserScan,int mapwidth,Point robot_pose){
    int x,y;
    int rposex=robot_pose.x+mapwidth/2;
    int rposey=robot_pose.y+mapwidth/2;
    int flag=0;
    int **map;
    map=new int* [mapwidth];
    for(y=0;y<mapwidth;y++){
        map[y]=new int [mapwidth];
        for(x=0;x<mapwidth;y++){
            map[y][x]=mapdata[y*mapwidth+x];
        }
    }
    for(y=(rposey-5);y<(rposey+5);y++){
        if(y>=(rposey-1) && y<=(rposey+1))
            continue;
        for(x=(rposex-5);x<(rposex+5);x++){
            if(y>=(rposey-1) && y<=(rposey+1))
                continue;

        }
    }
}*/

Pointf Positioning::transformto_masterodom(Pointf slave_pose){
    Pointf newp=Pointf();
    /*newp.x=slave_pose.x*cos(this->rotatedegree)-slave_pose.y*sin(this->rotatedegree)+diffx;
    newp.y=slave_pose.x*sin(this->rotatedegree)+slave_pose.y*cos(this->rotatedegree)+diffy;*/
    newp.x=slave_pose.x+diffx;
    newp.y=slave_pose.y+diffy;
    return newp;
}
