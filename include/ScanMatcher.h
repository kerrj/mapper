#ifndef SCANMATCHER_H
#define SCANMATCHER_H
#include <vector>
#include <string>
#include <iostream>
#include "ProbMap.h"
#include <list>
#include "math.h"
#include "ros/ros.h"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "mapper/Submap.h"
#include "mapper/RectifiedScan.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "mapper/Odometry.h"
#include "glog/logging.h"
#include "LaserScanCostEigen.h"
class ScanMatcher{
public:
	ScanMatcher();
	ProbMap resetMap();
	void addScan(const mapper::RectifiedScan::ConstPtr& scan,tf2_ros::TransformBroadcaster* br);
	void addOdom(const mapper::Odometry::ConstPtr& odom,tf2_ros::TransformBroadcaster* br);
	std::string getFrameId()const;
	mapper::Submap toRosMsg()const;
	ProbMap &getProbMap();
private:
	ProbMap map;
	geometry_msgs::TransformStamped getTrans(double x,double y,double th,std::string parent_name,std::string child_name)const;
	bool goodMeasurement(double x,double y);
	mapper::Odometry rPose;//integrates odometry/saves last estimate
	mapper::Odometry scanPose;//holds the delayed position of robot lining up with the scans
	std::list<mapper::Odometry::ConstPtr> odomQ;
	bool fresh=true;
	double MAX_RANGE=5.;//limit range we pay attention to measurements
	int id;
};
#endif
