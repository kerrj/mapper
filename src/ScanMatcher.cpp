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
	odomQ.push_back(odom);
}
bool ScanMatcher::goodMeasurement(double x,double y){
	if(isnan(x) || isinf(x))return false;
	double h=hypot(x,y);
	if(h<.15 || h>MAX_RANGE)return false;
	return true;
}
void ScanMatcher::addScan(const mapper::RectifiedScan::ConstPtr &scan,double* rx,double* ry,double* rth){
	//integrate the relevant odom messages
	while(odomQ.size()>0 && odomQ.front()->header.stamp<scan->header.stamp){
		double RT=rPose.th;
		double opt1=RT+odomQ.front()->th;
		double opt2=odomQ.front()->x/6.0;
		double opt3=odomQ.front()->th/2.0;
		double opt4=RT+opt3;
		double dx=opt2*(std::cos(RT)+4.0*std::cos(opt4)+std::cos(opt1));
		double dy=opt2*(std::sin(RT)+4.0*std::sin(opt4)+std::sin(opt1));
		rPose.x+=dx;
		rPose.y+=dy;
		rPose.th=opt1;
		odomQ.pop_front();
	}
	double p[]={rPose.x,rPose.y,rPose.th};
	if(!fresh){
		//do the pose optimization and set rx,ry,rth
		Problem problem;
		vector<double> xs;
		vector<double> ys;
		for(int i=0;i<scan->xs.size();i++){
			if(!goodMeasurement(scan->xs[i],scan->ys[i]))continue;
			xs.push_back(scan->xs[i]);
			ys.push_back(scan->ys[i]);
		}
		CostFunction *cost_fun=new AutoDiffCostFunction<LaserScanCost,DYNAMIC,3>(new LaserScanCost(&map,&xs,&ys),xs.size());
		problem.AddResidualBlock(cost_fun,new CauchyLoss(.9),p);
		Solver::Options options;
		options.num_threads=4;
		options.linear_solver_type=DENSE_QR;
		options.use_nonmonotonic_steps=true;
		options.minimizer_progress_to_stdout=false;
		Solver::Summary summary;
		Solve(options,&problem,&summary);
		std::cout<<summary.BriefReport()<<endl;
		*rx=p[0];
		*ry=p[1];
		*rth=p[2];
	}
	fresh=false;
	//add all the observations using the fine tuned pose
	for(int i=0;i<scan->xs.size();i++){
		if(!goodMeasurement(scan->xs[i],scan->ys[i]))continue;
		map.addObservation(p[0],p[1],p[2],scan->xs[i],scan->ys[i]);
	}
	//update the pose given the match
	rPose.x=p[0];rPose.y=p[1];rPose.th=p[2];
	
}
ProbMap ScanMatcher::getProbMap(){
	return map;
}
