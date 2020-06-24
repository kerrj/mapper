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
				BBNode node(xi,yi,thi,h0,x,y,th,T_RES,R_RES,points,map);
				res.push_back(node);
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
			BBNode newNode(xi+(1<<newH)*dx,yi+(1<<newH)*dy,thi,newH,rx,ry,rth,T_RES,R_RES,points,map);
			res.push_back(newNode);
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
	vector<vector<float> > maxMap=map->getMaxMap(height);
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
		score+=maxMap[gridX][gridY];
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
	*th=q.getAngle();
}
void GlobalMap::matchScan(Eigen::MatrixXf *points,ProbMap *map,double *x,double *y,double *th){
	const double T_RES=map->CELL_SIZE;//increment for translation
	const double R_RES=.017;//increment for rotation
	const double T_WINDOW=2;//meter
	const double R_WINDOW=.3;//rad
	const double SCORE_THRESH=points->cols()*.4;
	list<BBNode> stack=BBNode::getC0(T_WINDOW,R_WINDOW,T_RES,R_RES,*x,*y,*th,points,map);
	double best_score=0;
	int i=0;
	while(!stack.empty()){
		BBNode top=stack.back();
		stack.pop_back();
		if(top.getScore()<SCORE_THRESH)continue;
		if(top.getScore()>best_score){
			if(top.leaf()){
				cout<<"leaf properties"<<top.xi<<","<<top.yi<<","<<top.thi<<","<<top.getScore()<<endl;
				top.getPose(x,y,th);
				best_score=top.getScore();
			}else{
				list<BBNode> newNodes=top.branch();
				newNodes.sort();
				stack.splice(stack.end(),newNodes);
			}
		}
	}
}
