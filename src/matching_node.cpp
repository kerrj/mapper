#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/rotation.h"
#include "ProbMap.h"
#include "ScanMatcher.h"
#include "ros/ros.h"
#include "mapper/Odometry.h"
#include "mapper/RectifiedScan.h"
#include "mapper/ProbMap.h"
#include <vector>
#include <iostream>
using namespace ceres;
using namespace std;
ScanMatcher matcher;
ros::Publisher pub;
int num=0;
void scanCB(const mapper::RectifiedScan::ConstPtr &scan){
	double x,y,th;
	ros::Time start=ros::Time::now();
	matcher.addScan(scan,&x,&y,&th);
	ros::Duration elapse=ros::Time::now()-start;
	cout<<"matching: "<<elapse<<endl;
	pub.publish(matcher.getProbMap().toRosMsg());
}
void odomCB(const mapper::Odometry::ConstPtr& odom){
	matcher.addOdom(odom);
}
int main(int argc, char** argv){
	google::InitGoogleLogging(argv[0]);
	ros::init(argc,argv,"local_matching");
	ros::NodeHandle n;
	pub=n.advertise<mapper::ProbMap>("/map",10);
	ros::Subscriber sub1=n.subscribe("/rectified_scan",10,scanCB);
	ros::Subscriber sub2=n.subscribe("/wheel_odom",50,odomCB);
	ROS_INFO("Starting matching node");
	ros::spin();
	return 0;
}
