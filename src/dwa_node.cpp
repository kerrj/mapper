#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "geometry_msgs/TransformStamped.h"
#include "mapper/Path.h"
#include "mapper/EncoderReading.h"
#include <string>
#include "mapper/BaseCommand.h"
#include "mapper/RectifiedScan.h"
#include "Eigen/Dense"
#include <cmath>
#include <algorithm>
#include "Eigen/Geometry"
using namespace std;
const int RATE=10;//hz
const double SIM_TIME=5;//seconds
const double SIM_RES=.25;//frequency of samples in the forward simulation
const double LIN_ACC=.5;//units of m/s^2
const double ANG_ACC=6;//units of rad/s^2
const int LIN_SAMPLES=3;//discrete samples to take between min/max acc (centered at 0)
const int ANG_SAMPLES=21;//same as above for angle
const double WHEEL_RAD=.04;
const double WHEEL_SEP=.1978;

tf2_ros::Buffer tfBuf;
vector<pair<double,double> > lastScan;
vector<Eigen::Vector2d> lastPath;//path in robot coords
double curV, curW;
ros::Time lastScanTime(0);
ros::Time lastEncTime(0);

void scanCB(const mapper::RectifiedScan::ConstPtr &scan){
	lastScan.clear();
	lastScan.reserve(scan->xs.size());
	//convert to polar coords to make searching faster
	for(int i=0;i<scan->xs.size();i++){
		double th=atan2(scan->ys[i],scan->xs[i]);
		double dist=hypot(scan->ys[i],scan->xs[i]);
		lastScan.emplace_back(th,dist);
	}
	sort(lastScan.begin(),lastScan.end());
	lastScanTime=scan->header.stamp;
}

void pathCB(const mapper::Path::ConstPtr &path){
	//here we transform the path to robot coords and trim it to begin where we are currently
	if(!tfBuf.canTransform("submap_0","wheel_base",ros::Time(0),ros::Duration(.1))){
		ROS_WARN("Couldn't transform to submap 0 in pathCB");
		return;
	}
	lastPath.clear();
	lastPath.reserve(path->waypoints.size());
	geometry_msgs::TransformStamped robTrans=tfBuf.lookupTransform("wheel_base","submap_0",ros::Time(0));
	Eigen::Vector2d trans(robTrans.transform.translation.x,robTrans.transform.translation.y);
	double th,pitch,roll;
	tf2::Quaternion q;
	tf2::fromMsg(robTrans.transform.rotation,q);
	tf2::getEulerYPR(q,th,pitch,roll);
	Eigen::Rotation2D<double> R(th);
	//now transform all the points to the robot frame
	for(auto e:path->waypoints){
		Eigen::Vector2d p(e.x,e.y);
		Eigen::Vector2d newp=R*p+trans;
		lastPath.push_back(newp);
	}
}

void encCB(const mapper::EncoderReading::ConstPtr &msg){
	lastEncTime=msg->header.stamp;
	curV=(msg->leftVel+msg->rightVel)*WHEEL_RAD/2;
	curW=(msg->rightVel-msg->leftVel)*WHEEL_RAD/WHEEL_SEP;
}

mapper::BaseCommand getCommand(){
	//does everything lol
	mapper::BaseCommand cmd;
	cmd.omega=0;
	cmd.velocity=0;
	cmd.header.stamp=ros::Time::now();
	if(ros::Time::now()-lastScanTime>ros::Duration(.5) || ros::Time::now()-lastEncTime>ros::Duration(.1)){
		ROS_WARN("Stale data");
		return cmd;
	}
}

int main(int argc, char **argv){
	ros::init(argc,argv,"local_planning");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe("/scan",1,scanCB);
	ros::Subscriber sub2=n.subscribe("/path",1,pathCB);
	ros::Subscriber sub3=n.subscribe("/encoders",1,encCB);
	ros::Publisher commandPub=n.advertise<mapper::BaseCommand>("/target_vel",1);
	tf2_ros::TransformListener listener(tfBuf);
	ros::Rate rate(RATE);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
		mapper::BaseCommand cmd=getCommand();
		commandPub.publish(cmd);
	}
}
