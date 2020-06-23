#ifndef GLOBAL_MAP_H
#define GLOBAL_MAP_H
#include "ProbMap.h"
#include <vector>
#include <list>
#include <cmath>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/Transform.h"
#include "mapper/RectifiedScan.h"
#include "ceres/cubic_interpolation.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
using namespace std;
class BBNode{
public:
	double getScore();
	static list<BBNode> getC0(const double T_WINDOW,const double R_WINDOW,const double T_RES,const double R_RES,double x,double y,double th,Eigen::MatrixXd *points,ProbMap *map);
	bool operator<(BBNode &other){
		return getScore()<other.getScore();
	}
private:
	/* Constructor for creating the root
	 * x,y,th are start location of the node.
	 */
	BBNode(int xi,int yi,int thi,int height,double x,double y,double th,Eigen::MatrixXd *points,ProbMap *map);
	int xi,yi,thi,height;
	Eigen::MatrixXd *points;
	ProbMap *map;
	double rx,ry,rth;//start position of root
	double score=-1;
};
class GlobalMap{
public:
	GlobalMap();
	void addSubmap(ProbMap map);
	void findCorrespondences();//TODO define interface
	ProbMap getMap();
private:
	//x,y,th are position of addition in the base frame
	void mergeProbMaps(ProbMap &base,ProbMap &addition,double x,double y,double th);
	//points is 2xN in robot frame, x,y,th are in the given ProbMap frame, WILL BE MODIFIED BY ALG
	void matchScan(Eigen::MatrixXd *points,ProbMap *map,double *x,double *y,double *th);
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;//TODO init this in constructor
	vector<ProbMap> submaps;
};
#endif
