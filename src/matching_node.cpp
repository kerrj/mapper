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
	//cout<<"matching: "<<elapse<<endl;
	pub.publish(matcher.toRosMsg());
}
void odomCB(const mapper::Odometry::ConstPtr& odom){
	static tf2_ros::TransformBroadcaster br;
	matcher.addOdom(odom,&br);
}
bool resetCB(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &resp){
	//temporarily used as a test CB
	cout<<"testing max map"<<endl;
	//put test tings here
	double cx,cy,cth,x,y,th;
	vector<float> xs=lastScan->xs;
	vector<float> ys=lastScan->ys;
	Eigen::MatrixXf points(2,xs.size());
	points.row(0)=Eigen::Map<Eigen::MatrixXf>(xs.data(),1,xs.size());
	points.row(1)=Eigen::Map<Eigen::MatrixXf>(ys.data(),1,ys.size());
	for(float dx=-.1;dx<=.1;dx+=.01){
		for(float dy=-.1;dy<=.1;dy+=.01){
			for(float dth=-.25;dth<.25;dth+=.1){
				cout<<"testing offset "<<dx<<","<<dy<<","<<dth<<endl;
				gMap.getPose(&cx,&cy,&cth,"submap_0","last_scan");
				x=cx+dx;
				y=cy+dy;
				th=cth+dth;
				bool found=gMap.matchScan(&points,&matcher.getProbMap(),&x,&y,&th);
				if(!found){
					cout<<"no match found"<<endl;
				}
				double xerr=abs(x-cx);
				double yerr=abs(y-cy);
				double therr=abs(th-cth);
				if(xerr>.03 || yerr>.03 || therr>.01){
					cout<<"err: "<<xerr<<" , "<<yerr<<" , "<<therr<<endl;
				}
			}
		}
	}
	//end test things
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
