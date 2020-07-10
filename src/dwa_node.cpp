#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include <limits>
#include "tf2/utils.h"
#include "ProbMap.h"
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
const double SIM_TIME=3;//seconds
const double SIM_RES=.2;//frequency of samples in the forward simulation
const double LIN_ACC=.5;//units of m/s^2
const double ANG_ACC=12;//units of rad/s^2
const double MAX_LIN_VEL=.3;
const double MAX_ANG_VEL=3;
const int LIN_SAMPLES=3;//samples for HALF of the search space centered at 0. So 2 means 3 samples, 1 at min, 1 at 0, 1 at max;
const int ANG_SAMPLES=30;//same
const double WHEEL_RAD=.04;
const double WHEEL_SEP=.1978;
const double COL_RAD=.25/2.;
const double GOAL_TOL=.1;

tf2_ros::Buffer tfBuf;
vector<pair<double,double> > lastScan;
vector<Eigen::Vector2d> lastPath;//path in robot coords
double curV, curW;
ros::Time lastScanTime(0);
ros::Time lastEncTime(0);
int pathFinger=0;

void scanCB(const mapper::RectifiedScan::ConstPtr &scan){
	lastScan.clear();
	lastScan.reserve(scan->xs.size());
	//convert to polar coords to make searching faster
	for(int i=0;i<scan->xs.size();i++){
		double th=atan2(scan->ys[i],scan->xs[i]);
		double dist=hypot(scan->ys[i],scan->xs[i]);
		lastScan.emplace_back(dist,th);
	}
	sort(lastScan.begin(),lastScan.end());
	lastScanTime=scan->header.stamp;
}

void pathCB(const mapper::Path::ConstPtr &path){
	//here we transform the path to robot coords and trim it to begin where we are currently
	lastPath.clear();
	lastPath.reserve(path->waypoints.size());
	for(auto e:path->waypoints){
		Eigen::Vector2d p(e.x,e.y);
		lastPath.push_back(p);
	}
}

void encCB(const mapper::EncoderReading::ConstPtr &msg){
	lastEncTime=msg->header.stamp;
	curV=(msg->leftVel+msg->rightVel)*WHEEL_RAD/2;
	curW=(msg->rightVel-msg->leftVel)*WHEEL_RAD/WHEEL_SEP;
}
void closestPathPoint(double x,double y,double *pathX,double *pathY,vector<Eigen::Vector2d> &robPath){
	//doesnt ACTUALLY find closest path point, it searches along a short segment of the path
	//in front of the robot. robPath is the path in robot frame, pathFinger is set to the closest
	//path point
	double closestDist=numeric_limits<double>::max();
	//we start searching a little ways in front of the robot to prevent loop-back behavior of dwa
	const int START_ID=15;
	const int SEARCH_WIN=MAX_LIN_VEL*SIM_TIME/ProbMap::CELL_SIZE;
	for(int i=min(pathFinger+START_ID,(int)robPath.size()-1);i<robPath.size();i++){
		if(i>pathFinger+SEARCH_WIN)break;
		double dist=hypot(x-robPath[i](0),y-robPath[i](1));
		if(dist<closestDist){
			closestDist=dist;
			*pathX=robPath[i](0);
			*pathY=robPath[i](1);
		}
	}
}
bool collidesWithScan(double x,double y){
	//convert to polar
	double r=hypot(x,y);
	//bin search in scan to find nearest in distance
	auto closInd=lower_bound(lastScan.begin(),lastScan.end(),make_pair(r,0.));
	//consider a range of points around that distance corresponding to a disc shape of candidates
	for(auto tmp=closInd;tmp>=lastScan.begin() && tmp<lastScan.end();tmp++){
		//search outwards from start
		if(tmp->first>r+COL_RAD)break;
		double px=cos(tmp->second)*tmp->first;
		double py=sin(tmp->second)*tmp->first;
		if(hypot(px-x,py-y)<COL_RAD)return true;
	}
	for(auto tmp=closInd;tmp>=lastScan.begin() && tmp<lastScan.end();tmp--){
		//search inwards from start
		if(tmp->first<r-COL_RAD)break;
		double px=cos(tmp->second)*tmp->first;
		double py=sin(tmp->second)*tmp->first;
		if(hypot(px-x,py-y)<COL_RAD)return true;
	}
	return false;
}
double evalTraj(double v,double w,vector<Eigen::Vector2d> &robPath){
	//collisions have a score of infinity
	double x,y;
	for(double t=0;t<SIM_TIME;t+=SIM_RES){
		if(abs(w)<.0001){
			x=v*t;
			y=0;
		}else{
			x=(v/w)*sin(w*t);
			y=(v/w)-(v/w)*cos(w*t);
		}
		if(collidesWithScan(x,y))return numeric_limits<double>::max();
	}
	double pathX,pathY;
	closestPathPoint(x,y,&pathX,&pathY,robPath);
	double dist=hypot(x-pathX,y-pathY);
	return 1./(pow(v+.01,2))+.1*dist;
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
	if(lastPath.size()==0){
		ROS_WARN("0 length path");
		return cmd;
	}
	//first find the closest index on path to the current rob position while
	//transforming the path into robot frame
	if(!tfBuf.canTransform("submap_0","wheel_base",ros::Time(0),ros::Duration(.05))){
		ROS_WARN("Couldn't transform to submap 0 in pathCB");
		return cmd;
	}
	vector<Eigen::Vector2d> robPath;
	robPath.reserve(lastPath.size());
	geometry_msgs::TransformStamped robTrans=tfBuf.lookupTransform("wheel_base","submap_0",ros::Time(0));
	Eigen::Vector2d trans(robTrans.transform.translation.x,robTrans.transform.translation.y);
	double th,pitch,roll;
	tf2::Quaternion q;
	tf2::fromMsg(robTrans.transform.rotation,q);
	tf2::getEulerYPR(q,th,pitch,roll);
	Eigen::Rotation2D<double> R(th);
	//now transform all the points to the robot frame and find the closest to rob
	pathFinger=0;
	double bestDist=numeric_limits<double>::max();
	for(int i=0;i<lastPath.size();i++){
		auto e=lastPath[i];
		Eigen::Vector2d newp=R*e+trans;
		robPath.push_back(newp);
		double dist=hypot(newp(0),newp(1));
		if(dist<bestDist){
			pathFinger=i;
			bestDist=dist;
		}
	}
	if(hypot(robPath[robPath.size()-1](0),robPath[robPath.size()-1](1))<GOAL_TOL)return cmd;
	//generate space of trajs
	const double LIN_DELTA=(1./RATE)*LIN_ACC;
	const double ANG_DELTA=(1./RATE)*ANG_ACC;
	const double LIN_STEP=LIN_DELTA/LIN_SAMPLES;
	const double ANG_STEP=ANG_DELTA/ANG_SAMPLES;
	double bestScore=numeric_limits<double>::max();
	for(int vi=-LIN_SAMPLES;vi<LIN_SAMPLES;vi++){
		for(int wi=-ANG_SAMPLES;wi<ANG_SAMPLES;wi++){
			if(vi==0 && wi!=0)continue;
			double testV=vi*LIN_STEP+curV;
			double testW=wi*ANG_STEP+curW;
			if(testV<0 || testV>MAX_LIN_VEL || abs(testW)>MAX_ANG_VEL)continue;
			//for each traj: evaluate using score
			double score=evalTraj(testV,testW,robPath);
			//cout<<"evaling (v,w): "<<testV<<","<<testW<<" score: "<<score<<endl;
			if(score<bestScore){
				bestScore=score;
				cmd.velocity=testV;
				cmd.omega=testW;
			}
		}
	}
	return cmd;
}

int main(int argc, char **argv){
	ros::init(argc,argv,"local_planning");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe("/rectified_scan",1,scanCB);
	ros::Subscriber sub2=n.subscribe("/path",1,pathCB);
	ros::Subscriber sub3=n.subscribe("/encoders",1,encCB);
	ros::Publisher commandPub=n.advertise<mapper::BaseCommand>("/target_vel",1);
	tf2_ros::TransformListener listener(tfBuf);
	ros::Rate rate(RATE);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
		mapper::BaseCommand cmd=getCommand();
		cout<<"Given command (v,w): "<<cmd.velocity<<","<<cmd.omega<<endl;
		commandPub.publish(cmd);
	}
}
