#ifndef LASER_SCAN_COST_EIGEN_H
#define LASER_SCAN_COST_EIGEN_H
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "ceres/cubic_interpolation.h"
#include "ProbMap.h"

class LaserScanCostEigen{
public:
	LaserScanCostEigen(ProbMap* pm,std::vector<double>* pxs,std::vector<double>* pys){
		map=pm;
		interp=std::make_shared<ceres::BiCubicInterpolator<ProbMap> >(*map);
		points=Eigen::MatrixXd(2,pxs->size());
		points.row(0)=Eigen::Map<Eigen::MatrixXd>(pxs->data(),1,pxs->size());
		points.row(1)=Eigen::Map<Eigen::MatrixXd>(pys->data(),1,pys->size());
	}
	LaserScanCostEigen(ProbMap* pm,Eigen::MatrixXf* points){
		map=pm;
		interp=std::make_shared<ceres::BiCubicInterpolator<ProbMap> >(*map);
		this->points=points->cast<double>();
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
	std::shared_ptr<ceres::BiCubicInterpolator<ProbMap> > interp;
	ProbMap* map;
	Eigen::MatrixXd points;//2xN matrix
};
#endif

