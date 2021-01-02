#include "ScanMatcher.h"
#include "util.hpp"
#include "std_msgs/Float64.h"
using namespace ceres;
using namespace std;
#ifdef RVIZ_PUB
std::vector<sensor_msgs::PointField> pcfields(3);
void pubScan(string frame_id,ros::Publisher pub,const mapper::RectifiedScan::ConstPtr &scan){
	sensor_msgs::PointCloud2 pc;
	pc.header.stamp=scan->header.stamp;
	pc.header.frame_id=frame_id;
	pc.height=1;
	pc.width=scan->xs.size();
	pc.is_bigendian=true;
	pc.is_dense=true;
	pc.point_step=12;
	pc.row_step=pc.point_step*pc.width;
	pc.data=std::vector<uint8_t>(pc.row_step);
	pc.fields=pcfields;
	float* datastart=(float*)pc.data.data();
	for(int i=0;i<scan->xs.size();i++){
		datastart[3*i]=scan->xs[i];
		datastart[3*i+1]=scan->ys[i];	
	}
	pub.publish(pc);

}
#endif
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
	options.num_threads=4;
	options.max_num_iterations=25;
	options.linear_solver_type=DENSE_QR;
	options.use_nonmonotonic_steps=false;
	options.minimizer_progress_to_stdout=false;
#ifdef RVIZ_PUB	
	pcfields[0].name="x";
	pcfields[0].offset=0;
	pcfields[0].datatype=sensor_msgs::PointField::FLOAT32;
	pcfields[0].count=1;
	pcfields[1].name="y";
	pcfields[1].offset=4;
	pcfields[1].datatype=sensor_msgs::PointField::FLOAT32;
	pcfields[1].count=1;
	pcfields[2].name="z";
	pcfields[2].offset=8;
	pcfields[2].datatype=sensor_msgs::PointField::FLOAT32;
	pcfields[2].count=1;
#endif
}
ProbMap ScanMatcher::resetMap(){
	rPose.x=0;
	rPose.y=0;
	rPose.th=0;
	scanPose.x=0;
	scanPose.y=0;
	scanPose.th=0;
	map.crop();
	ProbMap cropped(map);
	cropped.crop();
	map=ProbMap();
	lastScanCost=100000000;
	fresh=true;
	id++;
	return cropped;
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
	if(h<.12 || h>MAX_RANGE)return false;
	return true;
}
bool ScanMatcher::addScan(const mapper::RectifiedScan::ConstPtr &scan,tf2_ros::TransformBroadcaster* br,
		ros::Publisher* pointcloudpub){
	//integrate the relevant odom messages bringing us up to the scan time
	while(odomQ.size()>0 && odomQ.front()->header.stamp<scan->header.stamp){
		doRKUpdate(scanPose,odomQ.front());
		odomQ.pop_front();
	}
	double p[]={scanPose.x,scanPose.y,scanPose.th};
	vector<double> xs;
	vector<double> ys;
	for(int i=0;i<scan->xs.size();i+=3){
		if(!goodMeasurement(scan->xs[i],scan->ys[i]))continue;
		xs.push_back(scan->xs[i]);
		ys.push_back(scan->ys[i]);
	}
	bool jumped=false;
	if(!fresh){
		//do the pose optimization and set rx,ry,rth
		Problem problem;
		CostFunction *cost_fun=new AutoDiffCostFunction<LaserScanCostEigen,DYNAMIC,3>(new LaserScanCostEigen(&map,&xs,&ys),xs.size());
		problem.AddResidualBlock(cost_fun,new ceres::HuberLoss(.7),p);
		Solver::Summary summary;
		Solve(options,&problem,&summary);
		//cout<<"pre: "<<summary.preprocessor_time_in_seconds<<" min: "<<summary.minimizer_time_in_seconds<<" post: "<<summary.postprocessor_time_in_seconds<<" tot: "<<summary.total_time_in_seconds<<endl;
		//lower cost is better
		if(summary.final_cost>1.5*lastScanCost){
			cout<<"jump detected: ";
			cout<<lastScanCost<<" -> "<<summary.final_cost<<endl;
			jumped=true;
		}
		lastScanCost=summary.final_cost;
#ifdef RVIZ_PUB
		static ros::NodeHandle n;
		static ros::Publisher costpub=n.advertise<std_msgs::Float64>("/match_cost",1);
		std_msgs::Float64 f;
		f.data=summary.final_cost;
		costpub.publish(f);
#endif
		//std::cout<<summary.BriefReport()<<endl;
	}
	fresh=false;
	scanPose.x=p[0];scanPose.y=p[1];scanPose.th=p[2];
#ifdef RVIZ_PUB
	if(pointcloudpub!=nullptr){
		geometry_msgs::TransformStamped b4Trans=getTrans(scanPose.x,scanPose.y,scanPose.th,getFrameId(),"scan_prematch");
		b4Trans.header.stamp=scan->header.stamp;
		br->sendTransform(b4Trans);
		pubScan("scan_prematch",*pointcloudpub,scan);
	}
#endif
	//add all the observations using the fine tuned pose
	if(!jumped){
		map.incScans(p[0],p[1]);
		xs.clear();
		ys.clear();
		for(int i=0;i<scan->xs.size();i+=1){
			if(!goodMeasurement(scan->xs[i],scan->ys[i]))continue;
			xs.push_back(scan->xs[i]);
			ys.push_back(scan->ys[i]);
		}
		map.addObservations(p[0],p[1],p[2],xs,ys);
	}
	//update the pose given the match
	string submapId=getFrameId();
	geometry_msgs::TransformStamped scanTrans=getTrans(scanPose.x,scanPose.y,scanPose.th,submapId,"last_scan");
	scanTrans.header.stamp=scan->header.stamp;
	br->sendTransform(scanTrans);
	//now update the robot pose based on relative position to last scan
	rPose.x=0;rPose.y=0;rPose.th=0;
	for(auto odom:odomQ){
		doRKUpdate(rPose,odom);
	}
	geometry_msgs::TransformStamped botTrans=getTrans(rPose.x,rPose.y,rPose.th,"last_scan","wheel_base");
	br->sendTransform(botTrans);
	return jumped;
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
nav_msgs::OccupancyGrid ScanMatcher::toNavMsg()const{
	nav_msgs::OccupancyGrid occg;
	occg.header.stamp=ros::Time::now();
	occg.header.frame_id=getFrameId();
	map.fillNavMsg(occg);
	return occg;
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
