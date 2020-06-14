#ifndef SCANMATCHER_H
#define SCANMATCHER_H
#include <vector>
#include <iostream>
#include "ProbMap.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/cubic_interpolation.h"
#include "glog/logging.h"
using namespace std;
using namespace ceres;
class ScanMatcher{

};
class LaserPointCost{
public:
	LaserPointCost(ProbMap pm,float px,float py){
		map=pm;
		interp=make_shared<BiCubicInterpolator<ProbMap> >(map);
		this->px=px;
		this->py=py;
	}
	template<typename T>
	bool operator()(const T* const x,const T* const y,const T* const th,T* residual)const{
		const T axis[]={T(0),T(0),*th};
        	const T point[]={T(px),T(py),T(0)};
        	T result[3];
        	AngleAxisRotatePoint(axis,point,result);
		T mx=result[0]+*x;
		T my=result[1]+*y;
		T gx,gy;
		map.map2Grid(mx,my,&gx,&gy);
        	interp->Evaluate(gx,gy,residual);
        	residual[0]=T(1.1)-residual[0];
        	return true;
	}
private:
	shared_ptr<BiCubicInterpolator<ProbMap> > interp;
	ProbMap map;
	double px,py;
};
#endif
