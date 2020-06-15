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
		poseFinger=0;
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
		pub.publish(rectscan);
		odomQ.clear();
		poseFinger=0;
	}
	void odomCB(const mapper::Odometry msg){
		odomQ.push_back(msg);
		//only keeps the last .4 seconds of updates after the finger
		while(poseFinger<odomQ.size()-1 && odomQ[poseFinger].header.stamp<odomQ[odomQ.size()-1].header.stamp-ros::Duration(.4)){
			poseFinger++;
		}
	}
	void rectify(const sensor_msgs::LaserScan::ConstPtr& scan,vector<float> &xs,
			vector<float> &ys){
		vector<Pose> poses=integrateOdom(scan);
		for(int i=0;i<scan->ranges.size();i++){
			float th=scan->intensities[i];
			float d=scan->ranges[i];
			if(isnan(d) || isinf(d) || d<.15)continue;
			Pose t=interp(poses,1-i/((float)scan->ranges.size()));
			Rotation2D<float> R(t.th);
			Vector2f original(d*cos(th),d*sin(th));
			Vector2f trans(t.x,t.y);
			Vector2f result=R*original+trans;
			xs.emplace_back(result(0));
			ys.emplace_back(result(1));
		}	
	}
private:
	vector<Pose> integrateOdom(const sensor_msgs::LaserScan::ConstPtr& scan){
		ros::Time scanStart=scan->header.stamp;
		/*cout<<"start integrate"<<endl;
		cout<<"scanstart "<<scanStart<<endl;
		cout<<"odomq size "<<odomQ.size()<<endl;
		cout<<"figner pos "<<poseFinger<<endl;*/
		int i=poseFinger;
		for( ;i<odomQ.size();i++){
			if(odomQ[i].header.stamp>scanStart){
				break;
			}
		}
		//cout<<"posefinger after increment "<<i<<endl;
		vector<Pose> poses={Pose(0,0,0)};
		for(;i<odomQ.size();i++){
			Pose prevP=poses[poses.size()-1];
			mapper::Odometry msg=odomQ[i];
			float RT=prevP.th;
			float opt1=RT+msg.th;
			float opt2=msg.x/6.0;
			float opt3=msg.th/2.0;
			float opt4=RT+opt3;
			float dx=opt2*(cos(RT)+4.0*cos(opt4)+cos(opt1));
			float dy=opt2*(sin(RT)+4.0*sin(opt4)+sin(opt1));
			poses.emplace_back(prevP.x+dx,prevP.y+dy,opt1);
		}
		return poses;
	}
	Pose interp(vector<Pose> &poses,float prog){
		if(poses.size()<2)return Pose(0,0,0);
		if(prog==1)return poses[poses.size()-1];
		int index=floor(prog*poses.size());
		if(index<0 || index>poses.size()-1){
			cout<<prog<<endl;
			cout<<poses.size()<<endl;
			throw runtime_error("index out of bounds");
		}
		if(index==poses.size()-1)return poses[poses.size()-1]; 
		float r=prog*poses.size()-index;
		float x=poses[index].x*(1-r)+poses[index+1].x*r;
		float y=poses[index].y*(1-r)+poses[index+1].y*r;
		float th=poses[index].th*(1-r)+poses[index+1].th*r;
		return Pose(x,y,th);
	}
	ros::Publisher pub;
	vector<mapper::Odometry> odomQ;
	int poseFinger=0;
	sensor_msgs::LaserScan::ConstPtr lastScan=nullptr;
};

int main(int argc, char **argv){
	ros::init(argc,argv,"rectify_scan");
        ros::NodeHandle n;
	ros::Publisher pub=n.advertise<mapper::RectifiedScan>("/rectified_scan",10);
	Rectifier rect(pub);
	ros::Subscriber s1 = n.subscribe("/scan",10,&Rectifier::scanCB,&rect);
	ros::Subscriber s2 = n.subscribe("/wheel_odom",50,&Rectifier::odomCB,&rect);
	ROS_INFO("Starting rectify node");
        ros::spin();
	return 0;
}
