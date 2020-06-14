#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/rotation.h"
#include "ProbMap.h"
#include "ScanMatcher.h"
#include <vector>
#include <iostream>
using namespace ceres;
using namespace std;

int main(int argc, char** argv){
	google::InitGoogleLogging(argv[0]);
	double x = 0;
	double y=0;
	double th=.3;
	vector<double> xs={0,0,0,2};
	vector<double> ys={0,1,2,2};
  	// Build the problem.
  	Problem problem;

  	// Set up the only cost function (also known as residual). This uses
  	// auto-differentiation to obtain the derivative (jacobian).
	ProbMap map;
	for(int i=0;i<xs.size();i++){
  		CostFunction* cost_function =
     		 new AutoDiffCostFunction<LaserPointCost, 1, 1,1,1>(new LaserPointCost(map,xs[i],ys[i]));
  		problem.AddResidualBlock(cost_function, NULL, &x,&y,&th);
	}

  	// Run the solver!
  	Solver::Options options;
	options.minimizer_type=LINE_SEARCH;
	options.minimizer_progress_to_stdout = true;
  	Solver::Summary summary;
  	Solve(options, &problem, &summary);

  	std::cout << summary.FullReport() << "\n";
  	std::cout << "x,y,th:" << x<<", "<<y <<", "<<th<< "\n";
	
	return 0;
}
