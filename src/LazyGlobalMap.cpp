#include "LazyGlobalMap.h"
using namespace std;
LazyGlobalMap::LazyGlobalMap(shared_ptr<tf2_ros::Buffer> buf){
	tfBuffer=buf;
	poses.emplace_back(3,0.);
}
void LazyGlobalMap::update(ProbMap m){
	if(poses.size()!=submaps.size()+1)cout<<"wrong poses/submap dims after update"<<endl;
	if(submaps.size()!=inflatedSubmaps.size())cout<<"wrong submap/inflatedsubmaps size after update"<<endl;
	current=ProbMap(m);
	current.crop();
	inflatedCurrent=inflateMap(current);
	for(int i=1;i<poses.size();i++){
		double* pose=(poses[i]).data();
		getPose(pose,pose+1,pose+2,"submap_"+to_string(i),"submap_0");
	}
	memoNormal=ProbMap();
	memoInflated=ProbMap();
}
void LazyGlobalMap::addSubmap(ProbMap m){
	m.crop();
	submaps.push_back(m);
	ProbMap inflated=inflateMap(m);
	inflatedSubmaps.push_back(inflated);
	poses.emplace_back(3,0.);//default for next map is 0, should be set in the next call to update
}
void LazyGlobalMap::getPose(double *x, double *y, double *th, string frame, string child_frame){
	geometry_msgs::TransformStamped t=tfBuffer->lookupTransform(frame,child_frame,ros::Time(0));
	getPose(x,y,th,t);
}
void LazyGlobalMap::getPose(double *x,double *y,double *th, geometry_msgs::TransformStamped t){
	*x=t.transform.translation.x;
	*y=t.transform.translation.y;
	tf2::Quaternion q;
	tf2::fromMsg(t.transform.rotation,q);
	double yaw,pitch,roll;
	tf2::getEulerYPR(q,yaw,pitch,roll);
	*th=yaw;
}
ProbMap LazyGlobalMap::inflateMap(ProbMap m){
	//TODO do this
	return m;
}
prob_t LazyGlobalMap::getInflated(double x,double y){
	//first check if it's memoized and return
	double gx,gy;
	memoInflated.map2Grid(x,y,&gx,&gy);
	int gridX=round(gx);
	int gridY=round(gy);
	prob_t memval=memoInflated.getProbT(gridX,gridY,true);
	if(memval>0){
		if(memval==1)return 0;//get rid of our weird flagging here
		return memval;
	}
	//otherwise we need to sample from every map and take the max
	prob_t maxval=0;
	for(int i=0;i<=inflatedSubmaps.size();i++){
		Eigen::Rotation2D<double> R(poses[i][2]);
		Eigen::Vector2d t(poses[i][0],poses[i][1]);
		Eigen::Vector2d p(x,y);
		//the above store transformation from submap_i frame to submap_0 frame
		Eigen::Vector2d transformed=R*p+t;
		double gx,gy;
		if(i==inflatedSubmaps.size()){
			inflatedCurrent.map2Grid(transformed(0),transformed(1),&gx,&gy);
		}else{
			inflatedSubmaps[i].map2Grid(transformed(0),transformed(1),&gx,&gy);
		}
		int gridX_tmp=round(gx);
		int gridY_tmp=round(gy);
		if(i==inflatedSubmaps.size()){
			maxval=max(maxval,inflatedCurrent.getProbT(gridX_tmp,gridY_tmp,true));
		}else{
			maxval=max(maxval,inflatedSubmaps[i].getProbT(gridX_tmp,gridY_tmp,true));
		}
	}
	if(maxval==0){
		memoInflated.setProbT(gridX,gridY,1);
		return 0;
	}
	memoInflated.setProbT(gridX,gridY,maxval);
	return maxval;
}
prob_t LazyGlobalMap::getNormal(double x,double y){
	//first check if it's memoized and return
	double gx,gy;
	memoNormal.map2Grid(x,y,&gx,&gy);
	int gridX=round(gx);
	int gridY=round(gy);
	prob_t memval=memoNormal.getProbT(gridX,gridY,true);
	if(memval>0){
		if(memval==1)return 0;//get rid of our weird flagging here
		return memval;
	}
	//otherwise we need to sample from every map and take the max
	prob_t maxval=0;
	for(int i=0;i<=submaps.size();i++){
		Eigen::Rotation2D<double> R(poses[i][2]);
		Eigen::Vector2d t(poses[i][0],poses[i][1]);
		Eigen::Vector2d p(x,y);
		//the above store transformation from submap_i frame to submap_0 frame
		Eigen::Vector2d transformed=R*p+t;
		double gx,gy;
		if(i==submaps.size()){
			current.map2Grid(transformed(0),transformed(1),&gx,&gy);
		}else{
			submaps[i].map2Grid(transformed(0),transformed(1),&gx,&gy);
		}
		int gridX_tmp=round(gx);
		int gridY_tmp=round(gy);
		if(i==submaps.size()){
			maxval=max(maxval,current.getProbT(gridX_tmp,gridY_tmp,true));
		}else{
			maxval=max(maxval,submaps[i].getProbT(gridX_tmp,gridY_tmp,true));
		}
	}
	if(maxval==0){
		memoNormal.setProbT(gridX,gridY,1);
		return 0;
	}
	memoNormal.setProbT(gridX,gridY,maxval);
	return maxval;

}
