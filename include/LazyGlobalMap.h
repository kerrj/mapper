#ifndef LAZY_GLOBAL_MAP_H
#define LAZY_GLOBAL_MAP_H
#include <vector>
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "ProbMap.h"
#include "RollingMax.h"
#include "RollingAverage.h"
#include "geometry_msgs/TransformStamped.h"
#include <string>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <cmath>
class LazyGlobalMap{
public:
	LazyGlobalMap(std::shared_ptr<tf2_ros::Buffer> buf);
	//updates the internal buffer of poses using tf2 data, functionally speaking
	//recalculates the new global map in a lazy way
	//m is the new currently building submap
	void update(ProbMap m);
	void addSubmap(ProbMap m);
	void getPose(double *x, double *y, double *th, std::string frame, std::string child_frame);
	void getPose(double *x, double *y, double *th, geometry_msgs::TransformStamped t);
	prob_t getInflated(double x,double y);
	prob_t getNormal(double x,double y);
	ProbMap getProbMap();
private:
	ProbMap inflateMap(const ProbMap m);
	std::vector<ProbMap> submaps;
	std::vector<std::vector<double> > poses;
	std::vector<ProbMap> inflatedSubmaps;
	ProbMap inflatedCurrent;
	ProbMap current;
	ProbMap memoNormal;
	ProbMap memoInflated;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer;
	const int BLUR_SIZE=21;
	const int INFLATE_SIZE=9;
};
#endif
