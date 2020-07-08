#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "ProbMap.h"
#include "tf2_ros/buffer.h"
#include "LazyGlobalMap.h"
#include "mapper/Submap.h"
#include "mapper/ProbMap.h"
#include "mapper/Path.h"
#include <string>
using namespace std;
shared_ptr<tf2_ros::Buffer> tfBuf=make_shared<tf2_ros::Buffer>();
LazyGlobalMap gmap(tfBuf);
ProbMap lastSubmap;
ProbMap submapToAdd;
string submapName="submap_0";
bool addSubmap=false;

void mapCB(const mapper::Submap::ConstPtr &msg){
	if(msg->header.frame_id!=submapName){
		addSubmap=true;
		submapName=msg->header.frame_id;
		submapToAdd=ProbMap(lastSubmap);
	}
	lastSubmap=ProbMap(msg->map);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"path_planner");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe("/submap",1,mapCB);
	ros::Publisher pub=n.advertise<mapper::Path>("/path",1);
	ros::Publisher pub2=n.advertise<mapper::ProbMap>("/inflated_map",1);
	tf2_ros::TransformListener listener(*tfBuf);
	ros::Rate rate(2);
	ROS_INFO("Starting path planning node");
	while(ros::ok()){
		//Danger, the probmaps we pass into the global map will be modified by global map
		rate.sleep();
		ros::spinOnce();
		ros::Time start=ros::Time::now();
		if(addSubmap){
			cout<<"adding new submap"<<endl;
			addSubmap=false;
			gmap.addSubmap(submapToAdd);
			gmap.update(lastSubmap);
			ProbMap inflatedglobal=gmap.getProbMap();
			mapper::ProbMap inflated_msg=inflatedglobal.toRosMsg();
			pub2.publish(inflated_msg);
		}else{
			gmap.update(lastSubmap);
		}
		cout<<"path loop time: "<<ros::Time::now()-start<<endl;
	}
	return 0;
}
