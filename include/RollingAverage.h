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
		vals.push_back(v);
		if(vals.size()>size){
			sum=sum-vals.front();
			vals.pop_front();
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
