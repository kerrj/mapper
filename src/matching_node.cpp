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
using namespace ceres;
using namespace std;
ScanMatcher matcher;
mapper::RectifiedScan::ConstPtr lastScan;
ros::Publisher pub;
shared_ptr<tf2_ros::Buffer> buf=make_shared<tf2_ros::Buffer>();
GlobalMap gMap(buf);
int num=0;
void scanCB(const mapper::RectifiedScan::ConstPtr &scan){
	lastScan=scan;
	static tf2_ros::TransformBroadcaster br;
	ros::Time start=ros::Time::now();
	matcher.addScan(scan,&br);
	ros::Duration elapse=ros::Time::now()-start;
	cout<<"matching: "<<elapse<<endl;
	pub.publish(matcher.toRosMsg());
}
void odomCB(const mapper::Odometry::ConstPtr& odom){
	static tf2_ros::TransformBroadcaster br;
	matcher.addOdom(odom,&br);
}
bool resetCB(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &resp){
	//temporarily used as a test CB
	cout<<"testing max map"<<endl;
	cout<<"map size: "<<matcher.getProbMap().numX()<<"x"<<matcher.getProbMap().numY()<<endl;
	ros::Time start=ros::Time::now();
	//put test tings here
	double x,y,th;
	vector<float> xs=lastScan->xs;
	vector<float> ys=lastScan->ys;
	gMap.getPose(&x,&y,&th,"submap_0","wheel_base");
	x+=1.1;
	Eigen::MatrixXf points(2,xs.size());
	points.row(0)=Eigen::Map<Eigen::MatrixXf>(xs.data(),1,xs.size());
	points.row(1)=Eigen::Map<Eigen::MatrixXf>(ys.data(),1,ys.size());
	cout<<"start pose"<<x<<","<<y<<","<<th<<endl;
	gMap.matchScan(&points,&matcher.getProbMap(),&x,&y,&th);
	cout<<"end pose"<<x<<","<<y<<","<<th<<endl;
	//end test things
	cout<<ros::Time::now()-start<<endl;
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
	tf2_ros::TransformListener listener(*buf);
	ROS_INFO("Starting matching node");
	ros::spin();
	return 0;
}
