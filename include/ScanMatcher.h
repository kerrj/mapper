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
#include "ceres/rotation.h"
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
using namespace std;
using namespace ceres;
class LaserScanCost{
public:
	LaserScanCost(ProbMap* pm,vector<double>* pxs,vector<double>* pys){
		map=pm;
		interp=make_shared<BiCubicInterpolator<ProbMap> >(*map);
		xs=pxs;
		ys=pys;
	}
	template<typename T>
	bool operator()(const T* const p,T* residual)const{
		const T axis[]={T(0),T(0),p[2]};
		for(int i=0;i<xs->size();i++){
        		const T point[]={T((*xs)[i]),T((*ys)[i]),T(0)};
        		T result[3];
        		AngleAxisRotatePoint(axis,point,result);
			T mx=result[0]+p[0];
			T my=result[1]+p[1];
			T gx,gy,interpRes;
			map->map2Grid(mx,my,&gx,&gy);
        		interp->Evaluate(gx,gy,&interpRes);
        		residual[i]=T(1.0)-interpRes;
		}
        	return true;
	}
private:
	shared_ptr<BiCubicInterpolator<ProbMap> > interp;
	ProbMap* map;
	vector<double>* xs;
	vector<double>* ys;
};
class LaserScanCostEigen{
public:
	LaserScanCostEigen(ProbMap* pm,vector<double>* pxs,vector<double>* pys){
		map=pm;
		interp=make_shared<BiCubicInterpolator<ProbMap> >(*map);
		points=Eigen::MatrixXd(2,pxs->size());
		points.row(0)=Eigen::Map<Eigen::MatrixXd>(pxs->data(),1,pxs->size());
		points.row(1)=Eigen::Map<Eigen::MatrixXd>(pys->data(),1,pys->size());
	}
	template<typename T>
	bool operator()(const T* const p,T* residual)const{
		Eigen::Rotation2D<T> rot(p[2]);
		Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> transformed=rot.toRotationMatrix()*points;
		Eigen::Matrix<T,2,1> trans(p[0],p[1]);
		transformed.colwise()+=trans;
		T one=T(1.0);
		for(int i=0;i<transformed.cols();i++){
			T gx,gy,interpRes;
			T mx=transformed(0,i);
			T my=transformed(1,i);
			map->map2Grid(mx,my,&gx,&gy);
        		interp->Evaluate(gx,gy,&interpRes);
        		residual[i]=one-interpRes;
		}
        	return true;
	}
private:
	shared_ptr<BiCubicInterpolator<ProbMap> > interp;
	ProbMap* map;
	Eigen::MatrixXd points;//2xN matrix
};
class ScanMatcher{
public:
	ScanMatcher();
	void resetMap();
	void addScan(const mapper::RectifiedScan::ConstPtr& scan,tf2_ros::TransformBroadcaster* br);
	void addOdom(const mapper::Odometry::ConstPtr& odom,tf2_ros::TransformBroadcaster* br);
	string getFrameId()const;
	mapper::Submap toRosMsg()const;
	ProbMap &getProbMap();
private:
	ProbMap map;
	geometry_msgs::TransformStamped getTrans(double x,double y,double th,string parent_name,string child_name)const;
	bool goodMeasurement(double x,double y);
	mapper::Odometry rPose;//integrates odometry/saves last estimate
	mapper::Odometry scanPose;//holds the delayed position of robot lining up with the scans
	list<mapper::Odometry::ConstPtr> odomQ;
	bool fresh=true;
	double MAX_RANGE=5.;//limit range we pay attention to measurements
	int id;
};
#endif
