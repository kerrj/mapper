#ifndef ROLLING_MAX_H
#define ROLLING_MAX_H
#include <list>
#include <limits>
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
	/*void add(T v){
		vals.push_back(v);
		if(vals.size()>size)vals.pop_front();
	}
	T getVal(){
		T maxval=0;
		for(auto e:vals){
			maxval=std::max(maxval,e);
		}
		return maxval;
	}*/
private:
	std::list<T> vals;
	std::list<int> ids;
	int size,id;
};
#endif
