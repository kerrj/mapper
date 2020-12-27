#ifndef UTIL_HPP
#define UTIL_HPP
#include <chrono>
#include <string>
#include <iostream>
inline std::chrono::time_point<std::chrono::system_clock> tic(){
	return std::chrono::system_clock::now();
}
inline void toc(std::string str,  std::chrono::time_point<std::chrono::system_clock> start){
	std::chrono::duration<double> elapse=std::chrono::system_clock::now()-start;
	std::cout<<"Secs for "<<str<<": "<<elapse.count()<<std::endl;
}
inline double warp(double v, double lo, double hi, double newlo, double newhi){
	double prog = (hi-v)/(hi-lo);
	return prog*newlo + (1-prog)*newhi;
}
#endif
