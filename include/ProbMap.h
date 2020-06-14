#ifndef PROBMAP_H
#define PROBMAP_H
#include <vector>
#include <iostream>
typedef uint8_t prob_t;
using namespace std;
class ProbMap{
public:
	ProbMap();
	ProbMap(const ProbMap &old);
	enum{DATA_DIMENSION=1};
	int rows()const;
	int cols()const;
	void GetValue(int row,int col,double* f)const;
	ProbMap& operator=(const ProbMap& other);
private:
	double getProb(prob_t p)const;
	double getProb(int row,int col) const;
	vector<vector<prob_t> > grid;
	const prob_t NO_INFO=50;
};
#endif
