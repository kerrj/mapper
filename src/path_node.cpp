#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "ProbMap.h"
#include "tf2_ros/buffer.h"
#include "LazyGlobalMap.h"
#include "mapper/Submap.h"
#include "geometry_msgs/TransformStamped.h"
#include "mapper/Path.h"
#include <string>
#include <queue>
#include "geometry_msgs/Point.h"
#include <unordered_set>
#include <algorithm>
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
class Node{
public:
	Node(int x,int y,double g){
		this->x=x;
		this->y=y;
		this->gval=g;
		parent=nullptr;
	}
	double getX(){
		return x*ProbMap::CELL_SIZE;
	}
	double getY(){
		return y*ProbMap::CELL_SIZE;
	}
	void setParent(shared_ptr<Node> par){
		parent=par;
	}
	double gval;
	int x,y;
	shared_ptr<Node> parent;
};
class NodeCmp{
public:
	bool operator()(const shared_ptr<Node> a,const shared_ptr<Node> b){
		return a->gval>b->gval;
	}
};
class PairHash{
public:
	size_t operator()(const pair<int,int> &p)const {
		return hash<int>()(p.first<<15+p.second);
	}
};
bool findPath(LazyGlobalMap &map,vector<geometry_msgs::Point> &path,int startx,int starty){
	priority_queue<shared_ptr<Node>,vector<shared_ptr<Node> >, NodeCmp> pq;
	pq.push(make_shared<Node>(startx,starty,0));
	unordered_set<pair<int,int>, PairHash> visited;
	shared_ptr<Node> goal=nullptr;
	int xd=0;
	while(!pq.empty()){
		shared_ptr<Node> next=pq.top();
		pq.pop();
		xd++;
		if(map.getInflated(next->getX(),next->getY())==0){
			//found an unobserved area, terminate
			goal=next;
			break;
		}
		//add neighbors
		for(int dx=-1;dx<=1;dx++){
			for(int dy=-1;dy<=1;dy++){
				if(dx==0 && dy==0)continue;
				if(map.getInflated(next->getX()+dx*ProbMap::CELL_SIZE,
							next->getY()+dy*ProbMap::CELL_SIZE)>200){
					//obstacle
					continue;
				}
				auto res=visited.find(make_pair(next->x+dx,next->y+dy));
				if(res!=visited.end()){
					//we've added this to pq already
					continue;
				}
				double cost=hypot(dx,dy);
				shared_ptr<Node> neigh=make_shared<Node>(next->x+dx,next->y+dy,
						next->gval+cost);
				neigh->setParent(next);
				pq.push(neigh);
				visited.emplace(next->x+dx,next->y+dy);
			}
		}
	}
	cout<<xd<<" nodes expanded"<<endl;
	if(goal==nullptr)return false;
	for(shared_ptr<Node> tmp=goal;tmp!=nullptr;tmp=tmp->parent){
		geometry_msgs::Point p;
		p.x=tmp->getX();
		p.y=tmp->getY();
		path.push_back(p);
	}
	reverse(path.begin(),path.end());
	return true;
}
int main(int argc, char** argv){
	ros::init(argc,argv,"path_planner");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe("/submap",1,mapCB);
	ros::Publisher pub=n.advertise<mapper::Path>("/path",1);
	tf2_ros::TransformListener listener(*tfBuf);
	ros::Rate rate(1);
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
		}
		gmap.update(lastSubmap);
		//now we can do thing with the planner
		vector<geometry_msgs::Point > path;
		if(!tfBuf->canTransform("submap_0","wheel_base",ros::Time(0),ros::Duration(.1))){
			ROS_WARN("Couldn't transform to find robot start pos");
			continue;
		}
		geometry_msgs::TransformStamped robPose=tfBuf->lookupTransform("submap_0","wheel_base",ros::Time(0));
		int startx=round(robPose.transform.translation.x/ProbMap::CELL_SIZE);
		int starty=round(robPose.transform.translation.y/ProbMap::CELL_SIZE);
		findPath(gmap,path,startx,starty);
		cout<<"path len "<<path.size()<<endl;
		mapper::Path pathmsg;
		pathmsg.waypoints=path;
		pathmsg.header.stamp=ros::Time::now();
		pathmsg.header.frame_id="submap_0";
		pub.publish(pathmsg);
		cout<<"path loop time: "<<ros::Time::now()-start<<endl;
	}
	return 0;
}
