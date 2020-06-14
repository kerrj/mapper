#include "ScanMatcher.h"
ScanMatcher::ScanMatcher(){
	rPose.x=0;
	rPose.y=0;
	rPose.th=0;
}
void ScanMatcher::resetMap(){
	map=ProbMap();
	fresh=true;
}
void ScanMatcher::addOdom(const mapper::Odometry::ConstPtr& odom){
	double RT=rPose.th;
	double opt1=RT+odom->th;
	double opt2=odom->x/6.0;
	double opt3=odom->th/2.0;
	double opt4=RT+opt3;
	double dx=opt2*(std::cos(RT)+4.0*std::cos(opt4)+std::cos(opt1));
	double dy=opt2*(std::sin(RT)+4.0*std::sin(opt4)+std::sin(opt1));
	rPose.x+=dx;
	rPose.y+=dy;
	rPose.th=opt1;
}
void ScanMatcher::addScan(const mapper::RectifiedScan::ConstPtr &scan,double* rx,double* ry,double* rth){
	double x=rPose.x;
	double y=rPose.y;
	double th=rPose.th;
	if(!fresh){
		x=0,y=0,th=.1;
		cout<<"sta pose: "<<x<<" "<<y<<" "<<th<<endl;
		//do the pose optimization and set rx,ry,rth
		Problem problem;
		for(int i=0;i<scan->xs.size();i++){
			if(isnan(scan->xs[i]) || isinf(scan->xs[i]))continue;
			CostFunction *cost_fun=new AutoDiffCostFunction<LaserPointCost,1,1,1,1>
							(new LaserPointCost(map,scan->xs[i],scan->ys[i]));
			problem.AddResidualBlock(cost_fun,NULL,&x,&y,&th);
		}
		Solver::Options options;
		//options.minimizer_type=LINE_SEARCH;
		options.num_threads=4;
		options.use_inner_iterations=true;
		options.minimizer_progress_to_stdout=true;
		Solver::Summary summary;
		Solve(options,&problem,&summary);
		std::cout<<summary.FullReport()<<endl;
		*rx=x;
		*ry=y;
		*rth=th;
		cout<<"end pose: "<<x<<" "<<y<<" "<<th<<endl;
	}
	fresh=false;
	//add all the observations using the fine tuned pose
	for(int i=0;i<scan->xs.size();i++){
		map.addObservation(x,y,th,scan->xs[i],scan->ys[i]);
	}
	//cout<<"map after update"<<endl;
	//for(int i=0;i<map.numX();i++){
	//	for(int j=0;j<map.numY();j++){
	//		cout<<map.getProb(i,j)<<" ";
	//	}
	//	cout<<endl;
	//}
}
