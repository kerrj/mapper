#include "ProbMap.h"

ProbMap::ProbMap(){
	int num_cells=DFLT_SIZE/CELL_SIZE;
	grid=make_shared<grid_t>(num_cells,vector<prob_t>(num_cells,NO_INFO));
	map_x=map_y=(num_cells/2)*CELL_SIZE;
}
ProbMap::ProbMap(const ProbMap& old){
	grid=old.grid;
	map_x=old.map_x;
	map_y=old.map_y;
}
int ProbMap::numX()const{
	return grid->size();
}
int ProbMap::numY()const{
	if(grid->size()<1)return 0;
	return (*grid)[0].size();
}
void ProbMap::GetValue(int x,int y,double* f)const{
	*f=getProb(x,y);
}
ProbMap& ProbMap::operator=(const ProbMap &other){
	if(this!=&other){
                grid=other.grid;
        }
        return *this;
}
double ProbMap::getProb(prob_t p)const{
	return ((double)p)/255.;
}
double ProbMap::getProb(int x,int y)const{
	if(x<0 || x>=numX() || y<0 || y>=numY()){
		return getProb(NO_INFO);
	}
	return getProb((*grid)[x][y]);
}
prob_t ProbMap::getProbT(double prob)const{
	int p=round(prob*255.);
	p=clamp(p,0,255);
	prob_t pt=p;
	return pt;
}
void ProbMap::resize(int num){
	if(num==0)return;
	int oldx=numX();
	int oldy=numY();
	shared_ptr<grid_t> newGrid=make_shared<grid_t>(2*num+grid->size(),vector<prob_t>(num,NO_INFO));
	//append on the middle section
	vector<prob_t> middleBlank(oldy,NO_INFO);
	vector<prob_t> endBlank(num,NO_INFO);
	for(int i=0;i<newGrid->size();i++){
		if(i<num || i>=num+grid->size()){
			(*newGrid)[i].insert((*newGrid)[i].end(),middleBlank.begin(),middleBlank.end());
		}else{
			(*newGrid)[i].insert((*newGrid)[i].end(),(*grid)[i-num].begin(),(*grid)[i-num].end());
		}
	}
	//append in the end section
	for(int i=0;i<newGrid->size();i++){
		(*newGrid)[i].insert((*newGrid)[i].end(),endBlank.begin(),endBlank.end());
	}
	grid=newGrid;
	map_x+=num*CELL_SIZE;
	map_y+=num*CELL_SIZE;
	if(oldx+2*num!=numX())throw runtime_error("wrong x dim");
	if(oldy+2*num!=numY())throw runtime_error("wrong y dim");
}
void ProbMap::addObservation(double rx,double ry,double rth,double px,double py){
	//first find the grid coords of the start and end points
	Eigen::Rotation2D<double> R(rth);
	Eigen::Vector2d trans(rx,ry);	
	Eigen::Vector2d robotPoint(px,py);
	Eigen::Vector2d mapPoint=R*robotPoint+trans;
	double gx,gy;
	map2Grid(mapPoint(0),mapPoint(1),&gx,&gy);
	int laserGridX=round(gx);
	int laserGridY=round(gy);
	int robotGridX=round(rx);
	int robotGridY=round(ry);
	//decide if we need to resize the map if they're out of bounds
	int pad=0;
	//take care of less than 0
	pad=max(pad,0-laserGridY);
	pad=max(pad,0-laserGridX);
	pad=max(pad,0-robotGridX);
	pad=max(pad,0-robotGridY);
	//and more than size of x,y
	pad=max(pad,laserGridX-numX());
	pad=max(pad,laserGridY-numY());
	pad=max(pad,robotGridX-numX());
	pad=max(pad,robotGridY-numY());
	resize(pad);
	//register the endpoints of the laser
	updateProb(laserGridX,laserGridY,false);
	updateProb(robotGridX,robotGridY,true);
	//call bresenham alg to paint between the endpoints with free space
	//TODO implement bresenham
}
double ProbMap::odds(double p){
	return p/(1-p);
}
double ProbMap::oddsinv(double p){
	return p/(1+p);
}
double ProbMap::clamp(double val,double minval,double maxval)const{
	return max(minval, min(val, maxval));
}
void ProbMap::updateProb(int x,int y,bool free){
	if(x<0 || x>=numX() || y<0 || y>=numY()){
		throw runtime_error("out of bounds access in updateProb");
	}
	double prior=getProb(x,y);
	double post;
	if(free){
		post=clamp(oddsinv(odds(prior)*odds(P_MISS)),0.,1.);
	}else{
		post=clamp(oddsinv(odds(prior)*odds(P_HIT)),0.,1.);
	}
	(*grid)[x][y]=getProbT(post);
}
