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
	double p[]={rPose.x,rPose.y,rPose.th};
	if(!fresh){
		cout<<"sta pose: "<<p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
		//do the pose optimization and set rx,ry,rth
		Problem problem;
		vector<double> xs;
		vector<double> ys;
		for(int i=0;i<scan->xs.size();i++){
			if(isnan(scan->xs[i]) || isinf(scan->xs[i]))continue;
			if(hypot(scan->xs[i],scan->ys[i])<.15)continue;
			xs.push_back(scan->xs[i]);
			ys.push_back(scan->ys[i]);
		}
		CostFunction *cost_fun=new AutoDiffCostFunction<LaserScanCost,DYNAMIC,3>(new LaserScanCost(&map,&xs,&ys),xs.size());
		problem.AddResidualBlock(cost_fun,NULL,p);
		Solver::Options options;
		//options.minimizer_type=LINE_SEARCH;
		options.num_threads=4;
		options.linear_solver_type=DENSE_QR;
		options.use_nonmonotonic_steps=true;
		options.minimizer_progress_to_stdout=true;
		Solver::Summary summary;
		Solve(options,&problem,&summary);
		std::cout<<summary.FullReport()<<endl;
		*rx=p[0];
		*ry=p[1];
		*rth=p[2];
		cout<<"end pose: "<<p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
	}
	fresh=false;
	//add all the observations using the fine tuned pose
	for(int i=0;i<scan->xs.size();i++){
		if(isnan(scan->xs[i]) || isinf(scan->xs[i]))continue;
		if(hypot(scan->xs[i],scan->ys[i])<.15)continue;
		map.addObservation(p[0],p[1],p[2],scan->xs[i],scan->ys[i]);
	}
}
void ScanMatcher::printMap(){
	map.printMap();
}
