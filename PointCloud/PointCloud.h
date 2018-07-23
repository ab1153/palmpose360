// PointCloud.h

#pragma once

#include <pcl/pcl_base.h>

extern "C"
{


class PalmPose
{
public:

	static __declspec(dllexport) int infer_palmpose(pcl::PointXYZ* v1, pcl::PointXYZ* v2, pcl::PointXYZ* v3, pcl::PointXYZ* p, int* size, pcl::PointXYZ* hand, pcl::PointXYZ* wrist);
	
};


}