#ifndef GLOBAL_MAP_H
#define GLOBAL_MAP_H
#include "ProbMap.h"
#include <vector>
#include <list>
#include <cmath>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ros/ros.h"
#include "geometry_msgs/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "mapper/RectifiedScan.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/ceres.h"
#include "LaserScanCostEigen.h"
#include "Eigen/Dense"
#include "geometry_msgs/PointStamped.h"
#include "Eigen/Geometry"
#include "ConstraintCost.h"
#include <limits>
using namespace std;
class BBNode{
public:
	/* Constructor for creating the root
	 * x,y,th are start location of the node.
	 */
	BBNode(int xi,int yi,int thi,int height,double x,double y,double th,const double T_RES,const double R_RES,Eigen::MatrixXf *points,ProbMap *map);
	double getScore();
	static list<BBNode> getC0(const double T_WINDOW,const double R_WINDOW,const double T_RES,const double R_RES,
			double x,double y,double th,Eigen::MatrixXf *points,ProbMap *map);
	list<BBNode> branch();
	void getPose(double *x,double *y,double *th);
	bool operator<(BBNode &other){
		return getScore()<other.getScore();
	}
	bool leaf();
	int xi,yi,thi,height;
private:
	Eigen::MatrixXf *points;
	ProbMap *map;
	double rx,ry,rth;//start position of root
	double score=-1;//-1 used as uninitialized
	double R_RES,T_RES;
};
class GlobalMap{
public:
	GlobalMap(std::shared_ptr<tf2_ros::Buffer> buf);
	void addSubmap(ProbMap map);
	bool matchScan(Eigen::MatrixXf *points,geometry_msgs::TransformStamped &trans,
			geometry_msgs::TransformStamped scan2cur);
	//points is 2xN in robot frame, x,y,th are in the given ProbMap frame, WILL BE MODIFIED BY ALG
	bool matchScan(Eigen::MatrixXf *points,ProbMap *map,double *x,double *y,double *th);
	ProbMap getMap();
	void getPose(double *x, double *y, double *th, std::string frame, std::string child_frame);
	void getPose(double *x, double *y, double *th,geometry_msgs::TransformStamped t);
	geometry_msgs::TransformStamped getTrans(double x,double y, double th,std::string parent_name, std::string child_name)const;
	int numMaps();
	void addConstraint(int parent, int child, double x,double y,double th,Eigen::Matrix3d covariance);
	void solve();
	void broadcastPoses(tf2_ros::StaticTransformBroadcaster &br);
private:
	std::shared_ptr<tf2_ros::Buffer> tfBuffer;
	std::vector<ProbMap> submaps;
	std::vector<std::vector<double> > poses;
	ceres::Problem problem;
	static constexpr double ZERO_THRESH=.2;
};
#endif
