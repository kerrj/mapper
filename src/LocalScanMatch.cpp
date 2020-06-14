#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/rotation.h"
#include <cmath>
#include <iostream>
#include <vector>
using namespace ceres;
using namespace std;
typedef uint8_t prob_t;
#define START .1
class ProbMap{
public:
	ProbMap(){
		//grid={{255,255,255},{255,0,255},{255,255,255}};
		grid={{255}};
	}
	ProbMap(const ProbMap &old){
		grid=old.grid;
	}
	enum{DATA_DIMENSION=1};
	int rows()const{
		return grid.size();
	}
	int cols()const{
		return grid[0].size();
	}
	void GetValue(int row,int col,double* f)const{
		*f=getProb(row,col);
	}
	double operator()(double x,double y){
		return getProb((int)x,(int)y);
	}
	ProbMap& operator=(const ProbMap &other){
		if(this!=&other){
			grid=other.grid;
		}
		return *this;
	}
private:
	double getProb(prob_t p)const {
		return ((double)p)/255.;
	}
	double getProb(int row,int col)const {
		//probability of out of map is 50/50
		if(row<0 || row>=rows() || col<0 || col>=cols()){
			return getProb(NO_INFO);
		}
		return getProb(grid[row][col]);
	}
	vector<vector<prob_t> > grid;
	const prob_t NO_INFO=127;
};
struct LaserPointCost {
	LaserPointCost(ProbMap pm,float px,float py){
		map=pm;
		interp=make_shared<BiCubicInterpolator<ProbMap> >(map);
		this->px=px;
		this->py=py;
		/*double p;
		for(int i=-10;i<11;i++){
			interp->Evaluate(0,i*.01,&p);
			cout<<p<<endl;
		}*/
	}
   template <typename T>
   bool operator()(const T* const x,const T* const y,const T* const th, T* residual) const {
	const T axis[]={T(0),T(0),*th};
	const T point[]={T(px),T(py),T(0)};
	T result[3];
	AngleAxisRotatePoint(axis,point,result);
     	interp->Evaluate(result[0]+*x,result[1]+*y,residual);
     	residual[0]=T(1.0)-residual[0];
     	return true;
   }
private:
   	shared_ptr<BiCubicInterpolator<ProbMap> > interp;
   	ProbMap map;
   	double px,py;//coords of the laser dot in robot frame
};

int main(int argc, char** argv){
	google::InitGoogleLogging(argv[0]);
	double x = 0;
	double y=START;
	double th=0;
  	// Build the problem.
  	Problem problem;

  	// Set up the only cost function (also known as residual). This uses
  	// auto-differentiation to obtain the derivative (jacobian).
	ProbMap map;
  	CostFunction* cost_function =
     	 new AutoDiffCostFunction<LaserPointCost, 1, 1,1,1>(new LaserPointCost(map,0,.1));
  	problem.AddResidualBlock(cost_function, NULL, &x,&y,&th);

  	// Run the solver!
  	Solver::Options options;
  	options.minimizer_progress_to_stdout = true;
  	Solver::Summary summary;
  	Solve(options, &problem, &summary);

  	std::cout << summary.FullReport() << "\n";
  	std::cout << "x,y,th:" << x<<", "<<y <<", "<<th<< "\n";
	
	return 0;
}
