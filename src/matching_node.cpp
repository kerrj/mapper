#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/cubic_interpolation.h"
#include "std_srvs/Trigger.h"
#include "ceres/rotation.h"
#include "ProbMap.h"
#include "ScanMatcher.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"
#include "Eigen/Dense"
#include "mapper/Odometry.h"
#include "mapper/RectifiedScan.h"
#include "mapper/AddSubmap.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf2_ros/transform_listener.h"
#include "mapper/ProbMap.h"
#include "GlobalMap.h"
#include "mapper/Submap.h"
#include <mutex>
#include <vector>
#include <iostream>
#include "util.hpp"
#include <chrono>
#include "util.hpp"
const double MAX_DIST_PER_SUBMAP=5;
const double SCAN_TIMEOUT=2;
using namespace std;
ScanMatcher matcher;
tf2_ros::Buffer buf;
ros::ServiceClient addMapClient;
ros::Publisher pub;
#ifdef RVIZ_PUB
ros::Publisher occ_grid_pub;
#endif
ros::Publisher pcpub;
double dist_travelled=0;
ros::Time lastScanTime(0);
mutex scan_lock;
void scanCB(const mapper::RectifiedScan::ConstPtr &scan){
	static int num=0;
	static tf2_ros::TransformBroadcaster br;
	static tf2_ros::StaticTransformBroadcaster statbr;
	static double sum=0;
	scan_lock.lock();
	lastScanTime=ros::Time::now();
	auto start = std::chrono::system_clock::now();
	bool reset=matcher.addScan(scan,&br,&pcpub); 
	std::chrono::duration<double> elapse = std::chrono::system_clock::now() - start;
	mapper::Submap sm=matcher.toRosMsg();
	pub.publish(sm);
#ifdef RVIZ_PUB
	//publish the OccupancyGrid here
	nav_msgs::OccupancyGrid occg=matcher.toNavMsg();
	occ_grid_pub.publish(occg);
#endif
	//if(dist_travelled>MAX_DIST_PER_SUBMAP || reset){//line integral
	if(hypot(matcher.scanPose.x,matcher.scanPose.y)>MAX_DIST_PER_SUBMAP || reset){//anchor
		dist_travelled=0;
		mapper::AddSubmap srv;
		srv.request.transform=buf.lookupTransform(matcher.getFrameId(),
				"last_scan",scan->header.stamp,ros::Duration(.01));
		geometry_msgs::TransformStamped t=buf.lookupTransform("submap_0","last_scan",scan->header.stamp,ros::Duration(.01));
		srv.request.map.header.frame_id=matcher.getFrameId();
		ProbMap frozenMap=matcher.resetMap();

		matcher.addScan(scan,&br);//this changes the parent of last_scan to the new map
		srv.request.map.map=frozenMap.toRosMsg();
		srv.request.map.header.stamp=ros::Time::now();
		srv.request.transform.child_frame_id=matcher.getFrameId();
		t.child_frame_id=matcher.getFrameId();
		statbr.sendTransform(t);
		if(addMapClient.call(srv))
			ROS_INFO("Dumped submap");
		else
			ROS_WARN("Couldn't dump submap");
	}
	scan_lock.unlock();
	sum+=elapse.count();
	num++;
	cout<<"Match time. ite: "<<elapse.count()<<" avg: "<<sum/num<<endl;
}
bool dumpSubmapCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp){
	static tf2_ros::TransformBroadcaster br;
	static tf2_ros::StaticTransformBroadcaster statbr;
	dist_travelled=0;
	mapper::AddSubmap srv;
	srv.request.transform=buf.lookupTransform(matcher.getFrameId(),
				"last_scan",ros::Time(0),ros::Duration(.01));
	geometry_msgs::TransformStamped t=buf.lookupTransform("submap_0","last_scan",ros::Time(0),ros::Duration(.01));
	srv.request.map.header.frame_id=matcher.getFrameId();
	ProbMap frozenMap=matcher.resetMap();
	srv.request.map.map=frozenMap.toRosMsg();
	srv.request.map.header.stamp=ros::Time::now();
	srv.request.transform.child_frame_id=matcher.getFrameId();
	t.child_frame_id=matcher.getFrameId();
	statbr.sendTransform(t);
	if(addMapClient.call(srv)){
		ROS_INFO("Dumped submap");
		resp.success=true;
		resp.message="Dumped current submap";
	}else{
		ROS_WARN("Couldn't dump submap");
		resp.success=false;
		resp.message="Couldn't dump current submap";
	}
	return true;
}
void odomCB(const mapper::Odometry::ConstPtr& odom){
	static tf2_ros::TransformBroadcaster br;
	matcher.addOdom(odom,&br);
	dist_travelled+=abs(odom->x);
}
int main(int argc, char** argv){
	ros::init(argc,argv,"local_matching");
	ros::NodeHandle n;
	addMapClient=n.serviceClient<mapper::AddSubmap>("/add_submap");
	tf2_ros::TransformListener list(buf);
	pub=n.advertise<mapper::Submap>("/submap",10);
#ifdef RVIZ_PUB
	occ_grid_pub=n.advertise<nav_msgs::OccupancyGrid>("/submap_occupancy",1);
	pcpub=n.advertise<sensor_msgs::PointCloud2>("/last_scan",1);
#endif
	ros::ServiceServer s = n.advertiseService("/dump_submap",dumpSubmapCB);
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
			ROS_WARN("No scan by local matcher in a while, shutting down node");
			ros::shutdown();
		}
		scan_lock.unlock();
	}
	return 0;
}
