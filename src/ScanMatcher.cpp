#include "ScanMatcher.h"
using namespace ceres;
using namespace std;
void doRKUpdate(mapper::Odometry &pose,const mapper::Odometry::ConstPtr &odom){
	double RT=pose.th;
	double opt1=RT+odom->th;
	double opt2=odom->x/6.0;
	double opt3=odom->th/2.0;
	double opt4=RT+opt3;
	double dx=opt2*(std::cos(RT)+4.0*std::cos(opt4)+std::cos(opt1));
	double dy=opt2*(std::sin(RT)+4.0*std::sin(opt4)+std::sin(opt1));
	pose.x+=dx;
	pose.y+=dy;
	pose.th=opt1;
}
ScanMatcher::ScanMatcher(){
	rPose.x=0;
	rPose.y=0;
	rPose.th=0;
	scanPose.x=0;
	scanPose.y=0;
	scanPose.th=0;
	id=0;
}
void ScanMatcher::resetMap(){
	rPose.x=0;
	rPose.y=0;
	rPose.th=0;
	scanPose.x=0;
	scanPose.y=0;
	scanPose.th=0;
	map=ProbMap();
	fresh=true;
	id++;
}
void ScanMatcher::addOdom(const mapper::Odometry::ConstPtr& odom,tf2_ros::TransformBroadcaster* br){
	odomQ.push_back(odom);
	doRKUpdate(rPose,odom);
	if(br!=nullptr){
		geometry_msgs::TransformStamped t=getTrans(rPose.x,rPose.y,rPose.th,"last_scan","wheel_base");
		br->sendTransform(t);
	}
}
bool ScanMatcher::goodMeasurement(double x,double y){
	if(isnan(x) || isinf(x))return false;
	double h=hypot(x,y);
	if(h<.15 || h>MAX_RANGE)return false;
	return true;
}
void ScanMatcher::addScan(const mapper::RectifiedScan::ConstPtr &scan,tf2_ros::TransformBroadcaster* br){
	//integrate the relevant odom messages bringing us up to the scan time
	while(odomQ.size()>0 && odomQ.front()->header.stamp<scan->header.stamp){
		doRKUpdate(scanPose,odomQ.front());
		odomQ.pop_front();
	}
	double p[]={scanPose.x,scanPose.y,scanPose.th};
	vector<double> xs;
	vector<double> ys;
	for(int i=0;i<scan->xs.size();i++){
		if(!goodMeasurement(scan->xs[i],scan->ys[i]))continue;
		xs.push_back(scan->xs[i]);
		ys.push_back(scan->ys[i]);
	}
	if(!fresh){
		//do the pose optimization and set rx,ry,rth
		Problem problem;
		CostFunction *cost_fun=new AutoDiffCostFunction<LaserScanCostEigen,DYNAMIC,3>(new LaserScanCostEigen(&map,&xs,&ys),xs.size());
		problem.AddResidualBlock(cost_fun,NULL,p);
		Solver::Options options;
		options.num_threads=4;
		options.linear_solver_type=DENSE_QR;
		options.use_nonmonotonic_steps=true;
		options.minimizer_progress_to_stdout=false;
		Solver::Summary summary;
		Solve(options,&problem,&summary);
		//std::cout<<summary.BriefReport()<<endl;
	}
	fresh=false;
	//add all the observations using the fine tuned pose
	for(int i=0;i<xs.size();i++){
		map.addObservation(p[0],p[1],p[2],xs[i],ys[i]);
	}
	//update the pose given the match
	scanPose.x=p[0];scanPose.y=p[1];scanPose.th=p[2];
	string submapId=getFrameId();
	geometry_msgs::TransformStamped scanTrans=getTrans(scanPose.x,scanPose.y,scanPose.th,submapId,"last_scan");
	//now update the robot pose based on relative position to last scan
	rPose.x=0;rPose.y=0;rPose.th=0;
	for(auto odom:odomQ){
		doRKUpdate(rPose,odom);
	}
	geometry_msgs::TransformStamped botTrans=getTrans(rPose.x,rPose.y,rPose.th,"last_scan","wheel_base");
	if(br==nullptr){
		ROS_WARN("No transform broadcaster given to addScan, skipping publishing poses");
		return;
	}
	br->sendTransform(scanTrans);
	br->sendTransform(botTrans);
}
ProbMap &ScanMatcher::getProbMap(){
	return map;
}
mapper::Submap ScanMatcher::toRosMsg()const{
	mapper::Submap msg;
	msg.header.stamp=ros::Time::now();
	msg.header.frame_id=getFrameId();
	msg.map=map.toRosMsg();
	return msg;
}
string ScanMatcher::getFrameId()const{
	return "submap_"+to_string(id);
}
geometry_msgs::TransformStamped ScanMatcher::getTrans(double x,double y,double th,string parent_name,string child_name)const{
	geometry_msgs::TransformStamped trans;
	trans.header.stamp=ros::Time::now();
	trans.header.frame_id=parent_name;
	trans.child_frame_id=child_name;
	trans.transform.translation.x=x;
	trans.transform.translation.y=y;
	trans.transform.translation.z=0;
	tf2::Quaternion q;
	q.setRPY(0,0,th);
	trans.transform.rotation.x=q.x();
	trans.transform.rotation.y=q.y();
	trans.transform.rotation.z=q.z();
	trans.transform.rotation.w=q.w();
	return trans;
}
