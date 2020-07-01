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
#include "mapper/AddSubmap.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "mapper/ProbMap.h"
#include "GlobalMap.h"
#include "mapper/Submap.h"
#include <mutex>
#include <vector>
#include <iostream>
const double MAX_DIST_PER_SUBMAP=5;
const double SCAN_TIMEOUT=2;
using namespace std;
ScanMatcher matcher;
tf2_ros::Buffer buf;
ros::ServiceClient addMapClient;
ros::Publisher pub;
double dist_travelled=0;
ros::Time lastScanTime(0);
mutex scan_lock;
void scanCB(const mapper::RectifiedScan::ConstPtr &scan){
	scan_lock.lock();
	lastScanTime=ros::Time::now();
	static tf2_ros::TransformBroadcaster br;
	ros::Time start=ros::Time::now();
	matcher.addScan(scan,&br);
	mapper::Submap sm=matcher.toRosMsg();
	pub.publish(sm);
	if(dist_travelled>MAX_DIST_PER_SUBMAP){
	//if(hypot(matcher.scanPose.x,matcher.scanPose.y)>5){
		dist_travelled=0;
		mapper::AddSubmap srv;
		srv.request.transform=buf.lookupTransform(matcher.getFrameId(),"last_scan",ros::Time(0));
		ProbMap frozenMap=matcher.resetMap();
		matcher.addScan(scan,&br);
		srv.request.map.map=frozenMap.toRosMsg();
		srv.request.map.header.stamp=ros::Time::now();
		srv.request.transform.child_frame_id=matcher.getFrameId();
		if(addMapClient.call(srv))
			ROS_INFO("Dumped submap");
		else
			ROS_WARN("Couldn't dump submap");
	}
	ros::Duration elapse=ros::Time::now()-start;
	cout<<"matching: "<<elapse<<endl;
	scan_lock.unlock();
}
void odomCB(const mapper::Odometry::ConstPtr& odom){
	static tf2_ros::TransformBroadcaster br;
	matcher.addOdom(odom,&br);
	dist_travelled+=abs(odom->x);
}
int main(int argc, char** argv){
	//google::InitGoogleLogging(argv[0]);
	ros::init(argc,argv,"local_matching");
	ros::NodeHandle n;
	addMapClient=n.serviceClient<mapper::AddSubmap>("/add_submap");
	tf2_ros::TransformListener list(buf);
	pub=n.advertise<mapper::Submap>("/submap",10);
	ros::Subscriber sub1=n.subscribe("/rectified_scan",50,scanCB);
	ros::Subscriber sub2=n.subscribe("/wheel_odom",200,odomCB);
	ROS_INFO("Starting matching node");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Rate rate(1);
	while(ros::ok()){
		rate.sleep();
		scan_lock.lock();
		if(lastScanTime>ros::Time(0) && ros::Time::now()-lastScanTime>ros::Duration(SCAN_TIMEOUT)){
			mapper::AddSubmap srv;
			srv.request.transform=buf.lookupTransform(matcher.getFrameId(),"last_scan",ros::Time(0));
			ProbMap frozenMap=matcher.resetMap();
			srv.request.map.map=frozenMap.toRosMsg();
			srv.request.map.header.stamp=ros::Time::now();
			srv.request.transform.child_frame_id=matcher.getFrameId();
			if(addMapClient.call(srv))
				ROS_INFO("Dumped submap");
			else
				ROS_WARN("Couldn't dump submap");
			spinner.stop();
			ros::shutdown();
		}
		scan_lock.unlock();
	}
	return 0;
}
