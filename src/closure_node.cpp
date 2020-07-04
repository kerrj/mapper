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
geometry_msgs::TransformStamped lastScanTrans;
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
	if(buf->canTransform("last_scan","submap_"+to_string(gmap.numMaps()),msg->header.stamp,ros::Duration(.5))){
		lastScanTrans=buf->lookupTransform("last_scan","submap_"+to_string(gmap.numMaps()),msg->header.stamp,
				ros::Duration(0));
	}else{
		lastScan=nullptr;
		ROS_WARN("Timed out waiting for next scan transform");
	}
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
	ros::Rate rate(2);
	ros::AsyncSpinner spinner(1,&serviceQueue);
	ROS_INFO("Starting closure node");
	spinner.start();
	while(ros::ok()){
		rate.sleep();
		bool publish=false;
		if(add){
			reqlock.lock();
			br.sendTransform(lastReq.transform);//new submap transform
			ProbMap map(lastReq.map.map);
			gmap.addSubmap(map);
			double x,y,th;
			gmap.getPose(&x,&y,&th,lastReq.transform);
			Eigen::DiagonalMatrix<double,3> covariance(.1,.1,.1);
			gmap.addConstraint(gmap.numMaps()-1,gmap.numMaps(),x,y,th,covariance);
			gmap.solve();
			gmap.broadcastPoses(br);
			add=false;
			reqlock.unlock();
			publish=true;
			ROS_INFO("Added submap");
		}
		ros::spinOnce();
		if(lastScan!=nullptr){
			//do the matching to the map here
			vector<float> xs;
			vector<float> ys;
			for(int i=0;i<lastScan->xs.size();i++){
				xs.push_back(lastScan->xs[i]);
				ys.push_back(lastScan->ys[i]);
			}
			if(xs.size()<500){
				ROS_INFO("not enough points to match");
			}else{
				Eigen::MatrixXf points(2,lastScan->xs.size());
				points.row(0)=Eigen::Map<Eigen::MatrixXf>(xs.data(),1,xs.size());
				points.row(1)=Eigen::Map<Eigen::MatrixXf>(ys.data(),1,ys.size());
				geometry_msgs::TransformStamped match;
				bool found=gmap.matchScan(&points,match,lastScanTrans);
				if(found){
					ROS_INFO("Adding match from %s to %s",match.header.frame_id.c_str(),
							match.child_frame_id.c_str());
					double x,y,th;
					gmap.getPose(&x,&y,&th,match);
					Eigen::DiagonalMatrix<double,3> covariance(.02,.02,.017);
					int parent=atoi(match.header.frame_id.c_str()+7);
					int child=atoi(match.child_frame_id.c_str()+7);
					gmap.addConstraint(parent,child,x,y,th,covariance);
					gmap.solve();
					gmap.broadcastPoses(br);
				}else{
					ROS_INFO("No match found");
				}
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
