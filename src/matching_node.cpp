#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/rotation.h"
#include "ProbMap.h"
#include "ScanMatcher.h"
#include "ros/ros.h"
#include "Eigen/Dense"
#include "mapper/Odometry.h"
#include "mapper/RectifiedScan.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "mapper/ProbMap.h"
#include "GlobalMap.h"
#include "mapper/Submap.h"
#include "std_srvs/Trigger.h"
#include <vector>
#include <iostream>
const double MAX_DIST_PER_SUBMAP=7;
using namespace std;
ScanMatcher matcher;
ros::Publisher pub;
double dist_travelled=0;
void scanCB(const mapper::RectifiedScan::ConstPtr &scan){
	static tf2_ros::TransformBroadcaster br;
	ros::Time start=ros::Time::now();
	matcher.addScan(scan,&br);
	mapper::Submap sm=matcher.toRosMsg();
	pub.publish(sm);
	ros::Duration elapse=ros::Time::now()-start;
	cout<<"matching: "<<elapse<<endl;
	//decide if we want to break the map
	if(dist_travelled>MAX_DIST_PER_SUBMAP){
		dist_travelled=0;
		cout<<"reseting map"<<endl;
	}
}
void odomCB(const mapper::Odometry::ConstPtr& odom){
	static tf2_ros::TransformBroadcaster br;
	matcher.addOdom(odom,&br);
	dist_travelled+=odom->x;
}
int main(int argc, char** argv){
	google::InitGoogleLogging(argv[0]);
	ros::init(argc,argv,"local_matching");
	ros::NodeHandle n;
	pub=n.advertise<mapper::Submap>("/submap",10);
	ros::Subscriber sub1=n.subscribe("/rectified_scan",10,scanCB);
	ros::Subscriber sub2=n.subscribe("/wheel_odom",100,odomCB);
	ROS_INFO("Starting matching node");
	ros::spin();
	return 0;
}
