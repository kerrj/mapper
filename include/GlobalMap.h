#include "ProbMap.h"
#include <vector>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/Transform.h"
using namespace std;
typedef int correspondence_t;//TODO define this properly
class GlobalMap{
public:
	GlobalMap();
	void addSubmap(ProbMap map);
	vector<correspondence_t> findCorrespondences(ProbMap map,double xwindow,double ywindow, double thwindow);
	ProbMap getMap();
private:
	//x,y,th are position of addition in the base frame
	void mergeProbMaps(ProbMap &base,ProbMap &addition,double x,double y,double th);
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;//TODO init this in constructor
	vector<ProbMap> submaps;

};
