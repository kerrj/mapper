#ifndef ROLLING_MAX_H
#define ROLLING_MAX_H
#include <list>
#include <limits>
using namespace std;
template<typename T>
class RollingMax{
public:
	RollingMax(int s){
		size=s;
		id=0;
	}
	void add(T v){
		while(!vals.empty() && vals.back() <= v){
			vals.pop_back();
			ids.pop_back();
		}
		vals.push_back(v);
		ids.push_back(id);
		while(ids.front() <= id-size){
			vals.pop_front();
			ids.pop_front();
		}
		id++;
	}
	T getVal(){
		return vals.front();
	}
private:
	list<T> vals;
	list<int> ids;
	int size,id;
};
#endif
