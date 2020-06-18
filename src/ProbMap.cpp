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
	num=max((int)(MIN_PAD/CELL_SIZE),num);
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
	map2Grid(rx,ry,&gx,&gy);
	int robotGridX=round(gx);
	int robotGridY=round(gy);
	//decide if we need to resize the map if they're out of bounds
	int pad=0;
	//take care of less than 0
	pad=max(pad,0-laserGridY);
	pad=max(pad,0-laserGridX);
	pad=max(pad,0-robotGridX);
	pad=max(pad,0-robotGridY);
	//and more than size of x,y
	pad=max(pad,laserGridX-numX()+1);
	pad=max(pad,laserGridY-numY()+1);
	pad=max(pad,robotGridX-numX()+1);
	pad=max(pad,robotGridY-numY()+1);
	resize(pad*2);
	map2Grid(mapPoint(0),mapPoint(1),&gx,&gy);
	laserGridX=round(gx);
	laserGridY=round(gy);
	map2Grid(rx,ry,&gx,&gy);
	robotGridX=round(gx);
	robotGridY=round(gy);
	//register the endpoints of the laser
	updateProb(laserGridX,laserGridY,P_HIT);
	updateProb(robotGridX,robotGridY,P_MISS);
	//call bresenham alg to paint between the endpoints with free space
	fillBetween(robotGridX,robotGridY,laserGridX,laserGridY);
}
void ProbMap::fillBetween(int x1,int y1,int x2,int y2)
{
 int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
 dx=x2-x1;
 dy=y2-y1;
 dx1=abs(dx);
 dy1=abs(dy);
 px=2*dy1-dx1;
 py=2*dx1-dy1;
 if(dy1<=dx1)
 {
  if(dx>=0)
  {
   x=x1;
   y=y1;
   xe=x2;
  }
  else
  {
   x=x2;
   y=y2;
   xe=x1;
  }
  for(i=0;x<xe;i++)
  {
   x=x+1;
   if(px<0)
   {
    px=px+2*dy1;
   }
   else
   {
    if((dx<0 && dy<0) || (dx>0 && dy>0))
    {
     y=y+1;
    }
    else
    {
     y=y-1;
    }
    px=px+2*(dy1-dx1);
   }
   if(x==x2 && y==y2)break;
   updateProb(x,y,P_MISS);
  }
 }
 else
 {
  if(dy>=0)
  {
   x=x1;
   y=y1;
   ye=y2;
  }
  else
  {
   x=x2;
   y=y2;
   ye=y1;
  }
  for(i=0;y<ye;i++)
  {
   y=y+1;
   if(py<=0)
   {
    py=py+2*dx1;
   }
   else
   {
    if((dx<0 && dy<0) || (dx>0 && dy>0))
    {
     x=x+1;
    }
    else
    {
     x=x-1;
    }
    py=py+2*(dx1-dy1);
   }
   if(x==x2 && y==y2)break;
   updateProb(x,y,P_MISS);
  }
 }
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
void ProbMap::updateProb(int x,int y,double update){
	if(x<0 || x>=numX() || y<0 || y>=numY()){
		cout<<x<<" "<<y<<endl;
		throw runtime_error("out of bounds access in updateProb");
	}
	double prior=getProb(x,y);
	double post;
	post=clamp(oddsinv(odds(prior)*odds(update)),0.05,.95);
	(*grid)[x][y]=getProbT(post);
}
void ProbMap::printMap(){
	double gx,gy;
	map2Grid(0.,0.,&gx,&gy);
	int gridx=round(gx);
	int gridy=round(gy);
	for(int i=0;i<numX();i++){
		for(int j=0;j<numY();j++){
			if(i==gridx && j==gridy){
				cout<<"O";
				continue;
			}
			double prob=getProb(i,j);
			if(prob>.5){
				cout<<"X";
			}else{
				cout<<" ";
			}
		}
		cout<<endl;
	}

}
mapper::ProbMap ProbMap::toRosMsg()const{
	mapper::ProbMap msg;
	msg.numX=numX();
	msg.numY=numY();
	msg.originX=round(map_x/CELL_SIZE);
	msg.originY=round(map_y/CELL_SIZE);
	msg.cellSize=CELL_SIZE;
	vector<uint8_t> data;
	for(int i=0;i<numX();i++){
		data.insert(data.end(),(*grid)[i].begin(),(*grid)[i].end());
	}
	msg.data=data;
	return msg;
}
