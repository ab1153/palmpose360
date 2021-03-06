// PointCloud.h

#pragma once

#include <pcl/pcl_base.h>

extern "C"
{


class PalmPose
{
public:

	static __declspec(dllexport) int infer_palmpose(pcl::PointXYZ* v1, pcl::PointXYZ* v2, pcl::PointXYZ* v3, pcl::PointXYZ* p, int* size, pcl::PointXYZ* hand, pcl::PointXYZ* wrist);
	// possibly add stateful implementation
	static __declspec(dllexport) void* init();
	static __declspec(dllexport) int uninit();
	static __declspec(dllexport) int estimate(void* ptr, 
		pcl::PointXYZ* v1, pcl::PointXYZ* v2, pcl::PointXYZ* v3, 
		pcl::PointXYZ* p, int* size, pcl::PointXYZ* hand, pcl::PointXYZ* wrist);
};


}