#include "GlobalMap.h"
using namespace std;
BBNode::BBNode(int xi,int yi,int thi,int height,double x,double y,double th,const double T_RES,const double R_RES,Eigen::MatrixXf *points,ProbMap *map){
	this->xi=xi;
	this->yi=yi;
	this->thi=thi;
	this->height=height;
	this->rx=x;
	this->ry=y;
	this->rth=th;
	this->T_RES=T_RES;
	this->R_RES=R_RES;
	this->points=points;
	this->map=map;
	this->score=-1;
}
list<BBNode> BBNode::getC0(const double T_WINDOW,const double R_WINDOW,const double T_RES,const double R_RES,
		double x,double y,double th,Eigen::MatrixXf *points,ProbMap *map){
	list<BBNode> res;
	const double MAX_TI=ceil(T_WINDOW/T_RES);//these are w_x,w_y,w_th in the paper
	const double MAX_RI=ceil(R_WINDOW/R_RES);
	const int h0=floor(log2(2*MAX_TI));
	for(int thi=-MAX_RI;thi<=MAX_RI;thi++){
		for(int jx=0;(1<<h0)*jx<=2*MAX_TI;jx++){
			for(int jy=0;(1<<h0)*jy<=2*MAX_TI;jy++){
				int xi = -MAX_TI + (1<<h0) * jx;
				int yi = -MAX_TI + (1<<h0) * jy;
				res.emplace_back(xi,yi,thi,h0,x,y,th,T_RES,R_RES,points,map);
			}
		}
	}
	res.sort();
	return res;
}
list<BBNode> BBNode::branch(){
	if(height==0)throw runtime_error("Branch() called on leaf node");
	list<BBNode> res;
	int newH=height-1;
	for(int dx=0;dx<2;dx++){
		for(int dy=0;dy<2;dy++){
			res.emplace_back(xi+(1<<newH)*dx,yi+(1<<newH)*dy,thi,newH,rx,ry,rth,T_RES,R_RES,points,map);
		}
	}
	return res;
}
void BBNode::getPose(double *x, double *y, double *th){
	*x=rx+xi*T_RES;
	*y=ry+yi*T_RES;
	*th=rth+thi*R_RES;
}
double BBNode::getScore(){
	if(score>=0)return score;
	//otherwise we need to compute and store
	vector<vector<float> > *maxMap=map->getMaxMap(height);
	score = 0;
	double sx,sy,sth;
	getPose(&sx,&sy,&sth);
	Eigen::Rotation2D<float> R(sth);
	Eigen::Vector2f trans(sx,sy);
	Eigen::MatrixXf transPoints=R.toRotationMatrix()*(*points);
	transPoints.colwise()+=trans;
	for(int i=0;i<transPoints.cols();i++){
		double gx,gy;
		double mx=transPoints(0,i);
		double my=transPoints(1,i);
		map->map2Grid(mx,my,&gx,&gy);
		int gridX=round(gx);
		int gridY=round(gy);
		if(gridX<0 || gridX>=map->numX() || gridY<0 || gridY>=map->numY())continue;
		score+=(*maxMap)[gridX][gridY];
	}
	return score;	
}
bool BBNode::leaf(){
	return height==0;
}
GlobalMap::GlobalMap(shared_ptr<tf2_ros::Buffer> buf){
	tfBuffer=buf;
}
void GlobalMap::getPose(double *x, double *y, double *th, string frame, string child_frame){
	geometry_msgs::TransformStamped t=tfBuffer->lookupTransform(frame,child_frame,ros::Time(0));
	*x=t.transform.translation.x;
	*y=t.transform.translation.y;
	tf2::Quaternion q;
	tf2::fromMsg(t.transform.rotation,q);
	double yaw,pitch,roll;
	tf2::getEulerYPR(q,yaw,pitch,roll);
	*th=yaw;
}
bool GlobalMap::matchScan(Eigen::MatrixXf *points,ProbMap *map,double *x,double *y,double *th){
	//const double T_RES=map->CELL_SIZE;//increment for translation
	const double T_RES=.03;
	const double R_RES=.006;//increment for rotation
	const double T_WINDOW=2;//meter
	const double R_WINDOW=.3;//rad
	list<BBNode> stack=BBNode::getC0(T_WINDOW,R_WINDOW,T_RES,R_RES,*x,*y,*th,points,map);
	double best_score=points->cols()*.6;
	bool found_match=false;
	while(!stack.empty()){
		BBNode top=stack.back();
		stack.pop_back();
		if(top.getScore()>=best_score){
			if(top.leaf()){
				found_match=true;
				top.getPose(x,y,th);
				best_score=top.getScore();
			}else{
				double sx,sy,sth;
				top.getPose(&sx,&sy,&sth);
				list<BBNode> newNodes=top.branch();
				newNodes.sort();
				stack.splice(stack.end(),newNodes);
			}
		}
	}
	return found_match;
	if(!found_match)return false;
	using namespace ceres;
	Problem problem;
	CostFunction *cost_fn=new AutoDiffCostFunction<LaserScanCostEigen,DYNAMIC,3>(new LaserScanCostEigen(map,points),points->cols());
	double p[]={*x,*y,*th};
	problem.AddResidualBlock(cost_fn,NULL,p);
	Solver::Options options;
	options.num_threads=4;
	options.linear_solver_type=DENSE_QR;
	options.use_nonmonotonic_steps=true;
	options.minimizer_progress_to_stdout=false;
	Solver::Summary summary;
	Solve(options,&problem,&summary);
	//if we drift too far from matcher mark as a warning
	*x=p[0];*y=p[1];*th=p[2];
	return true;
}
bool GlobalMap::matchScan(Eigen::MatrixXf *points,geometry_msgs::TransformStamped &trans){
	if(submaps.size()==0)return false;
	//choose the map whose center is closest to the scan
	if(!tfBuffer->canTransform("last_scan","submap_0",ros::Time(0),ros::Duration(.1))){
		ROS_WARN("Couldn't transform in matchScan");
		return false;
	}
	int besti=0;
	double bestdist=numeric_limits<double>::max();
	geometry_msgs::PointStamped scanOrigin;
	scanOrigin.header.frame_id="last_scan";
	geometry_msgs::PointStamped scanTrans;
	for(int i=0;i<submaps.size();i++){
		tfBuffer->transform(scanOrigin,scanTrans,"submap_"+to_string(i));
		double d=hypot(scanTrans.point.x,scanTrans.point.y);
		if(d<bestdist){
			bestdist=d;
			besti=i;
		}
	}
	double x,y,th;
	getPose(&x,&y,&th,"submap_"+to_string(besti),"last_scan");
	bool foundmatch=matchScan(points,&submaps[besti],&x,&y,&th);
	if(!foundmatch)return false;
	geometry_msgs::TransformStamped map2scan=getTrans(x,y,th,"submap_"+to_string(besti),"last_scan");
	geometry_msgs::TransformStamped scan2cur=tfBuffer->lookupTransform("last_scan","submap_"+
			to_string(submaps.size()),ros::Time(0));
	tf2::Transform map2scanT;
	tf2::convert(map2scan.transform,map2scanT);
	tf2::Transform scan2curT;
	tf2::convert(scan2cur.transform,scan2curT);
	//stupid shit below
	geometry_msgs::TransformStamped xd=tfBuffer->lookupTransform("submap_"+to_string(besti),"submap_"+to_string(submaps.size()),ros::Time(0));
	tf2::Transform can1=map2scanT*scan2curT;
	geometry_msgs::Transform printt;
	cout<<"TRANSFORMS BELOW"<<endl;
	cout<<xd<<endl;
	cout<<"submap_"+to_string(besti)<<endl;
	cout<<"submap_"+to_string(submaps.size())<<endl;
	tf2::convert(can1,printt);
	cout<<printt<<endl;
	/*tf2::Transform map2cur=map2scanT*scan2curT;
	tf2::convert(map2cur,trans.transform);
	trans.header.frame_id="submap_"+to_string(besti);
	trans.child_frame_id="submap_"+to_string(submaps.size());
	trans.header.stamp=ros::Time::now();*/
	return foundmatch;
}
void GlobalMap::addSubmap(ProbMap map,geometry_msgs::TransformStamped &transform){
	submaps.push_back(map);
}
ProbMap GlobalMap::getMap(){
	//this is janky for now to test things
	ProbMap map;
	double mx,my,gx,gy;
	ProbMap m=submaps[0];
	for(int xi=0;xi<m.numX();xi++){
		for(int yi=0;yi<m.numY();yi++){
			m.grid2Map(xi,yi,&mx,&my);
			map.resize(mx,my);
			map.map2Grid(mx,my,&gx,&gy);
			int gridx=round(gx);
			int gridy=round(gy);
			map.setProb(gridx,gridy,m.getProb(xi,yi,true));
		}
	}
	for(int i=1;i<submaps.size();i++){
		m=submaps[i];
		for(int xi=0;xi<m.numX();xi++){
			for(int yi=0;yi<m.numY();yi++){
				m.grid2Map(xi,yi,&mx,&my);
				geometry_msgs::PointStamped p;
				p.point.x=mx;
				p.point.y=my;
				p.point.z=0;
				p.header.frame_id="submap_"+to_string(i);
				tfBuffer->transform(p,p,"submap_0");
				map.resize(p.point.x,p.point.y);
				map.map2Grid(p.point.x,p.point.y,&gx,&gy);
				int gridx=round(gx);
				int gridy=round(gy);
				double newprob=max(m.getProb(xi,yi,true),map.getProb(gridx,gridy,true));
				map.setProb(gridx,gridy,newprob);
			}
		}
	}
	return map;
}
geometry_msgs::TransformStamped GlobalMap::getTrans(double x,double y,double th,string parent_name,string child_name)const{
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
