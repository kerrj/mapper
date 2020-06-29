#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/advertise_service_options.h"
#include "GlobalMap.h"
#include "ProbMap.h"
#include "mapper/AddSubmap.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "mapper/RectifiedScan.h"
#include <mutex>
const double MATCH_INTERVAL=2;
using namespace std;
mutex reqlock;;
shared_ptr<tf2_ros::Buffer> buf=make_shared<tf2_ros::Buffer>();
GlobalMap gmap(buf);
ros::Publisher mapPub;
mapper::AddSubmap::Request lastReq;
mapper::RectifiedScan::ConstPtr lastScan;
bool add=false;
bool submapCB(mapper::AddSubmap::Request &req,mapper::AddSubmap::Response  &res){
	reqlock.lock();
	lastReq=req;
	add=true;
	reqlock.unlock();
	res.success=true;
	return true;
}
void scanCB(const mapper::RectifiedScan::ConstPtr &msg){
	lastScan=msg;
}
int main(int argc, char** argv){
	ros::init(argc,argv,"loop_closure");
	ros::NodeHandle n;
	ros::CallbackQueue serviceQueue;

	ros::AdvertiseServiceOptions o=ros::AdvertiseServiceOptions::create<mapper::AddSubmap>("/add_submap",submapCB,ros::VoidPtr(),&serviceQueue);
	ros::ServiceServer ser=n.advertiseService(o);
	mapPub=n.advertise<mapper::ProbMap>("/map",1);
	ros::Subscriber sub=n.subscribe("/rectified_scan",1,scanCB);
	tf2_ros::StaticTransformBroadcaster br;
	tf2_ros::TransformListener list(*buf);
	ros::Rate rate(1);
	ros::AsyncSpinner spinner(1,&serviceQueue);
	ROS_INFO("Starting closure node");
	spinner.start();
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
		bool publish=false;
		if(add){
			reqlock.lock();
			br.sendTransform(lastReq.transform);
			ProbMap map(lastReq.map.map);
			gmap.addSubmap(map,lastReq.transform);
			add=false;
			reqlock.unlock();
			publish=true;
		}
		if(lastScan!=nullptr){
			//do the matching to the map here
			vector<float> xs=lastScan->xs;//have to make copies since lastScan is const
			vector<float> ys=lastScan->ys;
			Eigen::MatrixXf points(2,lastScan->xs.size());
			points.row(0)=Eigen::Map<Eigen::MatrixXf>(xs.data(),1,xs.size());
			points.row(1)=Eigen::Map<Eigen::MatrixXf>(ys.data(),1,ys.size());
			geometry_msgs::TransformStamped match;
			bool found=gmap.matchScan(&points,match);
			if(found){
				//cout<<match<<endl;
			}else{
				cout<<"no match found"<<endl;
			}
			lastScan=nullptr;
		}
		if(publish){
			ROS_INFO("Publishing global map...");
			ProbMap m=gmap.getMap();
			mapper::ProbMap mapMsg=m.toRosMsg();
			mapPub.publish(mapMsg);
			ROS_INFO("done");
		}
	}
	return 0;
}
