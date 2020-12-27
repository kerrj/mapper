#include "LazyGlobalMap.h"
#include <omp.h>
#include "util.hpp"
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
	if(!tfBuffer->canTransform("submap_"+to_string(poses.size()-1),"submap_0",ros::Time(0),
				ros::Duration(.1))){
		throw runtime_error("Timed out waiting for transform from submap 0 to current submap in lazy global map");
	}
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
	ProbMap inflated(m,true);//true means deepcopy the internal grid
	const double INFLATE_RAD  = .12;
	const double BLUR_RAD     = .4;
	const double INFLATE_RAD2 = std::pow(INFLATE_RAD,2);
	const double BLUR_RAD2    = std::pow(BLUR_RAD,2);
	const int inflate_window = std::ceil(BLUR_RAD/ProbMap::CELL_SIZE);
	for(int r=0;r<inflated.numX();r++){
		for(int c=0;c<inflated.numY();c++){
			prob_t maxval=0;
			double mindist=numeric_limits<double>::max();
			for(int dr = -inflate_window;dr<inflate_window+1;dr++){
				for(int dc = -inflate_window;dc<inflate_window+1;dc++){
					double d2 = std::pow(dr*ProbMap::CELL_SIZE,2)
						   + std::pow(dc*ProbMap::CELL_SIZE,2);
					prob_t mapval = m.getProbT(r+dr,c+dc);
					if(d2<=INFLATE_RAD2){
						if(mapval>maxval){
							maxval = mapval;
						}
					}else if(d2<=BLUR_RAD2 && mapval>=OBSTACLE_PROB){
						if(mindist>d2)mindist=d2;
					}
				}
			}
			if(maxval>=OBSTACLE_PROB || mindist == numeric_limits<double>::max()){
				inflated.setProbT(r,c,maxval);
			}else{ 
				double val = warp(mindist,INFLATE_RAD2,BLUR_RAD2,OBSTACLE_PROB,0);
				prob_t result = std::floor(val);
				inflated.setProbT(r,c,std::max(maxval,result));
			}
		}
	}
	return inflated;
	
}
/*ProbMap LazyGlobalMap::inflateMap(ProbMap m){
	ProbMap inflated=ProbMap(m);
	//now need to un-alias the data
	inflated.crop();//crop copies the data from the old buffer so we're good
	#pragma omp parallel for 
	for(int r=0;r<inflated.numX();r++){
		RollingMax<prob_t> rmax(INFLATE_SIZE);
		for(int c=0;c<inflated.numY();c++){
			rmax.add(inflated.getProbT(r,c));
			if(c>=INFLATE_SIZE/2){
				inflated.setProbT(r,c-INFLATE_SIZE/2,rmax.getVal());
			}
		}
	}
	#pragma omp parallel for 
	for(int c=0;c<inflated.numY();c++){
		RollingMax<prob_t> rmax(INFLATE_SIZE);
		for(int r=0;r<inflated.numX();r++){
			rmax.add(inflated.getProbT(r,c));
			if(r>=INFLATE_SIZE/2){
				inflated.setProbT(r-INFLATE_SIZE/2,c,rmax.getVal());
			}
		}
	}
	//blur the map some
	#pragma omp parallel for 
	for(int r=0;r<inflated.numX();r++){
		RollingAverage<int> ravg(BLUR_SIZE,0);
		for(int c=0;c<inflated.numY();c++){
			ravg.add(inflated.getProbT(r,c));
			if(c>=BLUR_SIZE/2){
				if(ravg.getVal()<inflated.getProbT(r,c-BLUR_SIZE/2,true))continue;
				inflated.setProbT(r,c-BLUR_SIZE/2,ravg.getVal());
			}
		}
	}
	#pragma omp parallel for 
	for(int c=0;c<inflated.numY();c++){
		RollingAverage<int> ravg(BLUR_SIZE,0);
		for(int r=0;r<inflated.numX();r++){
			ravg.add(inflated.getProbT(r,c));
			if(r>=BLUR_SIZE/2){
				if(ravg.getVal()<inflated.getProbT(r-BLUR_SIZE/2,c,true))continue;
				inflated.setProbT(r-BLUR_SIZE/2,c,ravg.getVal());
			}
		}
	}
	
	return inflated;
}
*/
prob_t LazyGlobalMap::getInflated(double x,double y){
	//first check if it's memoized and return
	memoInflated.resize(x,y);
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
	memoNormal.resize(x,y);
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
ProbMap LazyGlobalMap::getProbMap(){
	ProbMap map;
	for(int i=-500;i<500;i++){
		for(int j=-500;j<500;j++){
			double x=i*map.CELL_SIZE;
			double y=j*map.CELL_SIZE;
			map.resize(x,y);
			double gx,gy;
			map.map2Grid(x,y,&gx,&gy);
			int gridx=round(gx);
			int gridy=round(gy);
			prob_t val=getInflated(x,y);
			if(val<=50)continue;
			map.setProbT(gridx,gridy,val);
		}
	}
	return map;
}
