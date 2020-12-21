#ifndef PROBMAP_H
#define PROBMAP_H
#include <vector>
#include <iostream>
#include <memory>
#include "Eigen/Geometry"
#include "mapper/ProbMap.h"//this is the msg include
#include "mapper/Odometry.h"
#include "RollingMax.h"
#include "Eigen/Dense"
#include <math.h>
typedef uint8_t prob_t;
class ProbMap{
public:
	ProbMap();
	ProbMap(const ProbMap &old);
	ProbMap(mapper::ProbMap msg);
	void crop();
	enum{DATA_DIMENSION=1};
	void GetValue(int x,int y,double* f)const;
	//r{x,y,th} is the pose in the map frame, p{x,y} is the point in the robot frame
	void addObservation(double rx,double ry,double rth,double px,double py);
	void addObservations(double rx,double ry,double rth,std::vector<double> &xs, std::vector<double> &ys);
	mapper::ProbMap toRosMsg()const;
	template<typename T>
	void map2Grid(T mx,T my,T* gx,T* gy)const{
		*gx=(mx+T(map_x))*CELL_RES;
		*gy=(my+T(map_y))*CELL_RES;
	}
	void grid2Map(int gx,int gy,double* mx,double* my){
		*mx=gx*CELL_SIZE-map_x;
		*my=gy*CELL_SIZE-map_y;
	}
	std::vector<std::vector<prob_t> > *getMaxMap(int height);
	void resize(double x,double y);
	int numX()const;
	int numY()const;
	double getProb(prob_t p)const;
	prob_t getProbT(double p)const;
	prob_t getProbT(int x,int y,bool observability=false)const;
	double getProb(int x,int y,bool observability=false) const;
	void setProb(int x,int y,double p);
	void setProbT(int x,int y,prob_t p);
	static constexpr double CELL_SIZE=.04;
	static constexpr double PROB_MAX=.98;
	static constexpr double PROB_MIN=.05;
	void incScans(double rx,double ry);
	void getCOM(double &x,double &y);
private:
	void updateProb(int x,int y,double update);
	void fillBetween(int x0,int y0,int x1,int y1);
	std::vector<prob_t> rollRow(std::vector<prob_t> &row,int K);
	//adds num cells in every direction (new dims are (dx+2*num,dy+2*num))
	void resize(int num);
	double odds(double p);
	double oddsinv(double p);
	double clamp(double val,double minval,double maxval)const;
	std::shared_ptr<std::vector<std::vector<prob_t> > > grid;
	std::vector<std::vector<std::vector<prob_t> > > maxes;
	double map_x,map_y,sumX,sumY;
	int numScans;
	static constexpr prob_t NO_INFO=50;//NEVER MAKE THIS 0 since its reserved for keeping track of observed places
	static constexpr double CELL_RES=1/CELL_SIZE;
	static constexpr double DFLT_SIZE=10;
	static constexpr double MIN_PAD=5.;
	static constexpr double P_HIT=.52;
	static constexpr double P_MISS=.48;
};
#endif
