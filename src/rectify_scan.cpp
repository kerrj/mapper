#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "mapper/Odometry.h"
#include "mapper/RectifiedScan.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <vector>
#define PI 3.14159265358979323846
using namespace std;
using namespace Eigen;
const int SEAM_OFFSET=-8;
struct Pose{
	Pose(float x,float y,float th){
		this->x=x;
		this->y=y;
		this->th=th;
	}
	float x;
	float y;
	float th;
};
int mod(int x,int n){
	while(x<0)x+=n;
	return x%n;
}
class Rectifier{
public:
	Rectifier(ros::Publisher pub){
		this->pub=pub;
		Pose p(0,0,0);
		this->odomQ={p};
	}
	void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan){
		if(lastScan==nullptr){
			lastScan=scan;
		}
		lastScan=scan;
		mapper::RectifiedScan rectscan;
		rectscan.header.frame_id="wheel_base";
		rectscan.header.stamp=scan->header.stamp;
		rectscan.scan=*scan;
		vector<float> xs;
		vector<float> ys;
		rectify(scan,xs,ys);
		rectscan.xs=xs;
		rectscan.ys=ys;
		odomQ.clear();
		odomQ.emplace_back(0,0,0);
		pub.publish(rectscan);
	}
	void odomCB(const mapper::Odometry msg){
		if(lastScan==nullptr)return;
		if(msg.header.stamp<lastScan->header.stamp)return;
		Pose prevO=odomQ[odomQ.size()-1];
		odomQ.emplace_back(prevO.x+msg.x,prevO.y+msg.y,prevO.th+msg.th);
	}
	void rectify(const sensor_msgs::LaserScan::ConstPtr& scan,vector<float> &xs,
			vector<float> &ys){
		if(scan->ranges.size()!=720){
			ROS_WARN("Size of scan is not 720 points as expected");
		}
		for(int i=0;i<scan->ranges.size();i++){
			float th=((.5*i)*PI)/180.;
			float d=scan->ranges[i];
			Pose t=interp(odomQ,mod(i+2*SEAM_OFFSET,720)/720.f);
			Rotation2D<float> R(t.th);
			Vector2f original(d*cos(th),d*sin(th));
			Vector2f trans(t.x,t.y);
			Vector2f result=R*original+trans;
			xs.emplace_back(result(0));
			ys.emplace_back(result(1));
		}	
	}
private:
	Pose interp(vector<Pose> &poses,float prog){
		if(poses.size()<2)return Pose(0,0,0);
		int index=ceil(poses.size()*(1-prog));
		if(index>=poses.size())index=poses.size()-1;
		if(index<1 || index>poses.size()-1){
			cout<<prog<<endl;
			cout<<poses.size()<<endl;
			throw runtime_error("index out of bounds");
		}
		float r=prog*poses.size()-floor(prog*poses.size());
		if(r>1 || r<0){
			throw runtime_error("bad r");
		}
		float x=poses[index].x*(1-r)+poses[index-1].x*r;
		float y=poses[index].y*(1-r)+poses[index-1].y*r;
		float th=poses[index].th*(1-r)+poses[index-1].th*r;
		return Pose(x,y,th);
	}
	Pose imdumb(Pose p1,Pose p2,float r){
		return Pose(p1.x*(1-r)+p2.x*r,p1.y*(1-r)+p2.y*r,p1.th*(1-r)+p2.th*r);
	}
	ros::Publisher pub;
	vector<Pose> odomQ;
	sensor_msgs::LaserScan::ConstPtr lastScan=nullptr;
};

int main(int argc, char **argv){
	ros::init(argc,argv,"rectify_scan");
        ros::NodeHandle n;
	ros::Publisher pub=n.advertise<mapper::RectifiedScan>("/rectified_scan",10);
	Rectifier rect(pub);
	ros::Subscriber s1 = n.subscribe("/scan",10,&Rectifier::scanCB,&rect);
	ros::Subscriber s2 = n.subscribe("/wheel_odom",20,&Rectifier::odomCB,&rect);
	ROS_INFO("Starting rectify node");
        ros::spin();
	return 0;
}
