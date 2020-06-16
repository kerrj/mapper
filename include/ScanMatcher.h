#ifndef SCANMATCHER_H
#define SCANMATCHER_H
#include <vector>
#include <iostream>
#include "ProbMap.h"
#include <list>
#include "math.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/cubic_interpolation.h"
#include "mapper/RectifiedScan.h"
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
class ScanMatcher{
public:
	ScanMatcher();
	void resetMap();
	void addScan(const mapper::RectifiedScan::ConstPtr& scan,double *rx,double *ry,double *rth);
	void addOdom(const mapper::Odometry::ConstPtr& odom);
	ProbMap getProbMap();
private:
	ProbMap map;
	bool goodMeasurement(double x,double y);
	mapper::Odometry rPose;//integrates odometry/saves last estimate
	list<mapper::Odometry::ConstPtr> odomQ;
	bool fresh=true;
	double MAX_RANGE=7.;//limit range we pay attention to measurements
};
#endif
