#include "ProbMap.h"
using namespace std;
ProbMap::ProbMap(){
	int num_cells=DFLT_SIZE/CELL_SIZE;
	grid=make_shared<vector<vector<prob_t> > >(num_cells,vector<prob_t>(num_cells,0));
	map_x=map_y=(num_cells/2)*CELL_SIZE;
	sumX=sumY=0;
	numScans=0;
}
ProbMap::ProbMap(const ProbMap& old){
	grid=old.grid;
	map_x=old.map_x;
	map_y=old.map_y;
	sumX=old.sumX;
	sumY=old.sumY;
	numScans=old.numScans;
	maxes=old.maxes;
}
ProbMap::ProbMap(mapper::ProbMap msg){
	map_x=msg.originX;
	map_y=msg.originY;
	sumX=msg.sumX;
	numScans=msg.numScans;
	sumY=msg.sumY;
	grid=make_shared<vector<vector<prob_t> > >();
	for(int i=0;i<msg.numX;i++){
		auto it=msg.data.begin()+msg.numY*i;
		grid->emplace_back(it,it+msg.numY+1);
	}
}
void ProbMap::incScans(double rx,double ry){
	numScans++;
	sumX+=rx;
	sumY+=ry;
}
void ProbMap::getCOM(double &x,double &y){
	x=sumX/numScans;
	y=sumY/numScans;
}
void ProbMap::crop(){
	//shrinks the map to the size of the observed region
	int maxx=-1;int maxy=-1;
	int minx=numeric_limits<int>::max();int miny=numeric_limits<int>::max();
	//the above represent the range of VALID readings (inclusive)
	for(int x=0;x<numX();x++){
		for(int y=0;y<numY();y++){
			if(x>minx && x<maxx && y>miny && y<maxy)continue;
			if((*grid)[x][y]!=0){
				//if this location is observed, we need to make sure its 
				//inside the crop boundary
				maxx=max(x,maxx);
				maxy=max(y,maxy);
				minx=min(x,minx);
				miny=min(y,miny);
			}
		}
	}
	if(maxx==-1 || maxy==-1 || minx==numeric_limits<int>::max() || miny==numeric_limits<int>::max()){
		throw runtime_error("crop called on a blank map");
	}
	shared_ptr<vector<vector<prob_t> > > newGrid=make_shared<vector<vector<prob_t> > >();
	for(int x=minx;x<=maxx;x++){
		newGrid->emplace_back((*grid)[x].begin()+miny,(*grid)[x].begin()+maxy+1);
	}
	grid=newGrid;
	map_x-=minx*CELL_SIZE;
	map_y-=miny*CELL_SIZE;
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
double ProbMap::getProb(prob_t p)const{
	if(p==0)p=NO_INFO;
	return ((double)p)/255.;
}
double ProbMap::getProb(int x,int y,bool observability)const{
	if(x<0 || x>=numX() || y<0 || y>=numY()){
		return getProb(NO_INFO);
	}
	if(observability)return (*grid)[x][y]/255.;
	return getProb((*grid)[x][y]);
}
prob_t ProbMap::getProbT(double prob)const{
	int p=round(prob*255.);
	p=clamp(p,0,255);
	prob_t pt=p;
	return pt;
}
void ProbMap::setProb(int x,int y,double p){
	if(x<0 || x>=numX() || y<0 || y>=numY()){
		cout<<x<<" "<<y<<endl;
		throw runtime_error("out of bounds access in setProb");
	}
	prob_t prob=getProbT(p);
	(*grid)[x][y]=prob;
}
void ProbMap::resize(int num){
	if(num==0)return;
	num=max((int)(MIN_PAD/CELL_SIZE),num);
	int oldx=numX();
	int oldy=numY();
	shared_ptr<vector<vector<prob_t> > > newGrid=make_shared<vector<vector<prob_t> > >(2*num+grid->size(),vector<prob_t>(num,0));
	//append on the middle section
	vector<prob_t> middleBlank(oldy,0);
	vector<prob_t> endBlank(num,0);
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
void ProbMap::resize(double x,double y){
	double gx,gy;
	map2Grid(x,y,&gx,&gy);
	int gridX=round(gx);
	int gridY=round(gy);
	int pad=max(0,0-gridY);
	pad=max(pad,0-gridX);
	pad=max(pad,gridX-numX()+1);
	pad=max(pad,gridY-numY()+1);
	resize(pad);
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
	resize(pad);
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
	double post=clamp(oddsinv(odds(prior)*odds(update)),PROB_MIN,PROB_MAX);
	(*grid)[x][y]=getProbT(post);
}
mapper::ProbMap ProbMap::toRosMsg()const{
	mapper::ProbMap msg;
	msg.numX=numX();
	msg.numY=numY();
	msg.originX=map_x;
	msg.originY=map_y;
	msg.sumX=sumX;
	msg.sumY=sumY;
	msg.numScans=numScans;
	msg.cellSize=CELL_SIZE;
	vector<uint8_t> data;
	for(int i=0;i<numX();i++){
		data.insert(data.end(),(*grid)[i].begin(),(*grid)[i].end());
	}
	msg.data=data;
	return msg;
}

vector<vector<float> > *ProbMap::getMaxMap(int height){
	if(height<0)throw runtime_error("negative height requested in getMaxMap");
	if(height<maxes.size())return &maxes[height];
	for(int map=maxes.size();map<=height;map++){
		vector<vector<float> > maxMap;
		const int wSize=1<<map;
		//first compute the max for rows of wSize
		for(int x=0;x<numX();x++){
			vector<float> newRow=rollRow((*grid)[x],wSize);
			maxMap.push_back(newRow);
		}
		//then go up columns to accumulate maxes of the rows
		for(int y=0;y<numY();y++){
			RollingMax<float> r(wSize);
			for(int x=numX()-1;x>=0;x--){
				double v=maxMap[x][y];
				if(v<=.2)v=0;
				r.add(maxMap[x][y]);
				maxMap[x][y]=r.getVal();
			}
		}
		maxes.push_back(maxMap);
	}
	return &maxes[height];
}
vector<float> ProbMap::rollRow(vector<prob_t> &row, int size) {
	RollingMax<float> r(size);
	vector<float> res;
	res.resize(row.size());
	for(int i=row.size()-1;i>=0;i--){
		r.add(getProb(row[i]));
		res[i]=r.getVal();
	}
	return res;

}
