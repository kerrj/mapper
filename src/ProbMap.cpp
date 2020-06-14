#include "ProbMap.h"

ProbMap::ProbMap(){
	grid={{255,255,255},{50,50,255},{0,0,255}};
}
ProbMap::ProbMap(const ProbMap& old){
	grid=old.grid;
}
int ProbMap::rows()const{
	return grid.size();
}
int ProbMap::cols()const{
	if(grid.size()<1)return 0;
	return grid[0].size();
}
void ProbMap::GetValue(int row,int col,double* f)const{
	*f=getProb(row,col);
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
double ProbMap::getProb(int row,int col)const{
	if(row<0 || row>=rows() || col<0 || col>=cols()){
		return getProb(NO_INFO);
	}
	return getProb(grid[row][col]);
}
