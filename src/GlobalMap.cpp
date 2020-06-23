#include "GlobalMap.h"
BBNode::BBNode(int xi,int yi,int thi,int height,double x,double y,double th,Eigen::MatrixXd *points,ProbMap *map){
	this->xi=xi;
	this->yi=yi;
	this->thi=thi;
	this->height=height;
	this->rx=x;
	this->ry=y;
	this->rth=th;
	this->points=points;
	this->map=map;
	score=-1;
}
list<BBNode> BBNode::getC0(const double T_WINDOW,const double R_WINDOW,const double T_RES,const double R_RES,
		double x,double y,double th,Eigen::MatrixXd *points,ProbMap *map){
	list<BBNode> res;
	const double MAX_TI=ceil(T_WINDOW/T_RES);//these are w_x,w_y,w_th in the paper
	const double MAX_RI=ceil(R_WINDOW/R_RES);
	const int h0=floor(log2(2*MAX_TI));
	for(int thi=-MAX_RI;thi<=MAX_RI;thi++){
		for(int jx=0;(1<<h0)*jx<=2*MAX_TI;jx++){
			for(int jy=0;(1<<h0)*jy<=2*MAX_TI;jy++){
				int xi = -MAX_TI + (1<<h0) * jx;
				int yi = -MAX_TI + (1<<h0) * jy;
				BBNode node(xi,yi,thi,h0,x,y,th,points,map);
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
			BBNode newNode(xi+(1<<newH)*dx,yi+(1<<newH)*dy,thi,newH,rx,ry,rth,points,map);
			res.push_back(newNode);
		}
	}
	return res;
}
double BBNode::getScore(){
	if(score>=0)return score;
	//otherwise we need to compute and store
	return 0;
}
void GlobalMap::matchScan(Eigen::MatrixXd *points,ProbMap *map,double *x,double *y,double *th){
	const double T_RES=map->CELL_SIZE;//increment for translation
	const double R_RES=acos(1-(pow(T_RES,2)/(2*pow(5,2))));//5 is the max range (approx is ok)
	const double T_WINDOW=2;//meter
	const double R_WINDOW=.5;//rad
	list<BBNode> stack=BBNode::getC0(T_WINDOW,R_WINDOW,T_RES,R_RES,*x,*y,*th,points,map);
}
