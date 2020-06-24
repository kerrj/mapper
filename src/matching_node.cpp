#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/rotation.h"
#include "ProbMap.h"
#include "ScanMatcher.h"
#include "ros/ros.h"
#include "mapper/Odometry.h"
#include "mapper/RectifiedScan.h"
#include "tf2_ros/transform_broadcaster.h"
#include "mapper/ProbMap.h"
#include "mapper/Submap.h"
#include "std_srvs/Trigger.h"
#include <vector>
#include <iostream>
using namespace ceres;
using namespace std;
ScanMatcher matcher;
ros::Publisher pub;
bool test=false;
int num=0;
void scanCB(const mapper::RectifiedScan::ConstPtr &scan){
	static tf2_ros::TransformBroadcaster br;
	ros::Time start=ros::Time::now();
	matcher.addScan(scan,&br);
	ros::Duration elapse=ros::Time::now()-start;
	cout<<"matching: "<<elapse<<endl;
	if(test){
		//do some stuff here
	}
	pub.publish(matcher.toRosMsg());
}
void odomCB(const mapper::Odometry::ConstPtr& odom){
	static tf2_ros::TransformBroadcaster br;
	matcher.addOdom(odom,&br);
}
bool resetCB(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &resp){
	//temporarily used as a test CB
	test=true;
	/*cout<<"testing max map"<<endl;
	cout<<"map size: "<<matcher.getProbMap().numX()<<"x"<<matcher.getProbMap().numY()<<endl;
	ros::Time start=ros::Time::now();
	matcher.getProbMap().getMaxMap(8);
	cout<<ros::Time::now()-start<<endl;
	*/
	//matcher.resetMap();
	resp.success=true;
	return true;
}
int main(int argc, char** argv){
	google::InitGoogleLogging(argv[0]);
	ros::init(argc,argv,"local_matching");
	ros::NodeHandle n;
	pub=n.advertise<mapper::Submap>("/submap",10);
	ros::Subscriber sub1=n.subscribe("/rectified_scan",10,scanCB);
	ros::Subscriber sub2=n.subscribe("/wheel_odom",50,odomCB);
	ros::ServiceServer ss=n.advertiseService("/reset_submap",resetCB);
	ROS_INFO("Starting matching node");
	ros::spin();
	return 0;
}
