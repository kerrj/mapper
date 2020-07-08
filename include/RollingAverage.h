#ifndef ROLLING_AVERAGE_H
#define ROLLING_AVERAGE_H
#include <list>
template<typename T>
class RollingAverage{
public:
	RollingAverage(int s,T zero){
		size=s;
		sum=zero;
	}
	void add(T v){
		sum=sum+v;
		if(vals.size()>size){
			sum=sum-vals.front();
			vals.pop_frontront();
		}
	}
	T getVal(){
		return sum/vals.size();
	}
private:
	std::list<T> vals;
	T sum;
	int size;
};
#endif
