#ifndef PROBMAP_H
#define PROBMAP_H
#include <vector>
#include <iostream>
#include <memory>
#include "Eigen/Geometry"
#include "mapper/ProbMap.h"
#include "Eigen/Dense"
#include <math.h>
using namespace std;
typedef uint8_t prob_t;
typedef vector<vector<prob_t> > grid_t;
class ProbMap{
public:
	ProbMap();
	ProbMap(const ProbMap &old);
	enum{DATA_DIMENSION=1};
	void GetValue(int x,int y,double* f)const;
	//r{x,y,th} is the pose in the map frame, p{x,y} is the point in the robot frame
	void addObservation(double rx,double ry,double rth,double px,double py);
	void fromRosMsg(const mapper::ProbMap::ConstPtr &msg);
	mapper::ProbMap toRosMsg()const;
	template<typename T>
	void map2Grid(T mx,T my,T* gx,T* gy)const{
		*gx=(mx+T(map_x))*CELL_RES;
		*gy=(my+T(map_y))*CELL_RES;
	}
	ProbMap& operator=(const ProbMap& other);
	int numX()const;
	int numY()const;
	double getProb(prob_t p)const;
	prob_t getProbT(double p)const;
	double getProb(int x,int y) const;
	void printMap();
private:
	void updateProb(int x,int y,double update);
	void fillBetween(int x0,int y0,int x1,int y1);
	//adds num cells in every direction (new dims are (dx+2*num,dy+2*num))
	void resize(int num);
	double odds(double p);
	double oddsinv(double p);
	double clamp(double val,double minval,double maxval)const;
	shared_ptr<vector<vector<prob_t> > > grid;
	double map_x,map_y;
	const prob_t NO_INFO=50;
	const double CELL_SIZE=.05;
	const double CELL_RES=1/CELL_SIZE;
	const double DFLT_SIZE=10;
	const double MIN_PAD=2.;
	const double P_HIT=.52;
	const double P_MISS=.48;
};
#endif
