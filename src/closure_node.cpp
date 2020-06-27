#include "ros/ros.h"
#include "GlobalMap.h"
#include "ProbMap.h"
#include "mapper/AddSubmap.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "mapper/RectifiedScan.h"
const double MATCH_INTERVAL=2;
using namespace std;
shared_ptr<tf2_ros::Buffer> buf=make_shared<tf2_ros::Buffer>();
GlobalMap gmap(buf);
ros::Publisher mapPub;
bool submapCB(mapper::AddSubmap::Request &req,mapper::AddSubmap::Response &res){
	static tf2_ros::StaticTransformBroadcaster br;
	br.sendTransform(req.transform);
	ProbMap map(req.map.map);
	gmap.addSubmap(map,req.transform);
	ProbMap m=gmap.getMap();
	mapper::ProbMap mapMsg=m.toRosMsg();
	mapPub.publish(mapMsg);
	return true;
}
void scanCB(const mapper::RectifiedScan::ConstPtr &msg){
	//msg queue is 1 so this is the most recent scan;
}
int main(int argc, char** argv){
	ros::init(argc,argv,"loop_closure");
	ros::NodeHandle n;
	ros::ServiceServer ser=n.advertiseService("/add_submap",submapCB);
	mapPub=n.advertise<mapper::ProbMap>("/map",1);
	ros::Subscriber sub=n.subscribe("/rectified_scan",1,scanCB);
	tf2_ros::TransformListener list(*buf);
	ROS_INFO("Starting closure node");
	ros::spin();
	return 0;
}
