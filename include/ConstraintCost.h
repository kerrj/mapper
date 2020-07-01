#ifndef CONSTRAINT_COST_H
#define CONSTRAINT_COST_H
#include "Eigen/Dense"
#include "Eigen/Geometry"
const double PI=3.14159265358979323846;
class ConstraintCost{
public:
	ConstraintCost(double x,double y,double th,Eigen::Matrix3d covariance){
		invCovariance=covariance.inverse();
		this->x=x;
		this->y=y;
		this->th=th;
	}
	template<typename T>
	T angleDiff(T t1,T t2)const{
		T diff=t2-t1;
		while(diff>=T(2*PI)){
			diff-=T(2*PI);
		}
		while(diff<T(0)){
			diff+=T(2*PI);
		}
		if(diff>T(PI)){
			diff-=T(2*PI);
		}
		return diff;
	}
	template<typename T>
	bool operator()(const T* const p1,const T* const p2,T* residual)const{
		T thErr=angleDiff(T(th),angleDiff(p1[2],p2[2]));
		Eigen::Rotation2D<T> rot(p1[2]);
		Eigen::Matrix<T,2,1> t(x,y);
		Eigen::Matrix<T,2,1> trans=rot*t;
		Eigen::Matrix<T,2,1> p2predict(p1[0]+trans(0),p1[1]+trans(1));
		Eigen::Matrix<T,3,1> err(p2predict(0)-p2[0],p2predict(1)-p2[1],thErr);
		Eigen::Matrix<T,1,1> res=err.transpose()*invCovariance*err;
		residual[0]=res(0,0);
		return true;
	}
	Eigen::Matrix3d invCovariance;
	double x,y,th;//stores the pose of p2 in the p1 frame
};
class ConstraintCostOrigin{
public:
	ConstraintCostOrigin(double x,double y,double th,Eigen::Matrix3d covariance){
		invCovariance=covariance.inverse();
		this->x=x;
		this->y=y;
		this->th=th;
	}
	template<typename T>
	T angleDiff(T t1,T t2)const{
		T diff=t2-t1;
		while(diff>=T(2*PI)){
			diff-=T(2*PI);
		}
		while(diff<T(0)){
			diff+=T(2*PI);
		}
		if(diff>T(PI)){
			diff-=T(2*PI);
		}
		return diff;
	}
	template<typename T>
	bool operator()(const T* const p2,T* residual)const{
		T thErr=angleDiff(T(th),p2[2]);
		Eigen::Matrix<T,2,1> p2predict(x,y);
		Eigen::Matrix<T,3,1> err(p2predict(0)-p2[0],p2predict(1)-p2[1],thErr);
		Eigen::Matrix<T,1,1> res=err.transpose()*invCovariance*err;
		residual[0]=res(0,0);
		return true;
	}
	Eigen::Matrix3d invCovariance;
	double x,y,th;//stores the pose of p2 in the p1 frame
};
#endif
