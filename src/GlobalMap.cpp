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
	/*
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
		int winsize=(1<<height);
		double maxval=0;
		for(int lx=0;lx<winsize;lx++){
			for(int ly=0;ly<winsize;ly++){
				if(gridX+lx<0 || gridX+lx>=map->numX() || gridY+ly<0 || gridY+ly>=map->numY())continue;
				maxval=max(maxval,map->getProb(gridX+lx,gridY+ly));
			}
		}
		score+=maxval;
	}
	return score;*/

	
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
	double best_score=points->cols()*.5;
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
				//cout<<"node "<<top.xi+100<<","<<top.yi+100<<","<<top.height<<" pos: "<<","<<sx<<","<<sy<<" score "<<top.getScore()<<endl;
				list<BBNode> newNodes=top.branch();
				/*cout<<"children: ";
				for(auto b:newNodes){
					cout<<"("<<b.xi+100<<","<<b.yi+100<<")";
				}
				cout<<endl;*/
				newNodes.sort();
				stack.splice(stack.end(),newNodes);
			}
		}
	}
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
