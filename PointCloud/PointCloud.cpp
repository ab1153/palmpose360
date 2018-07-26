#include "stdafx.h"

#include <pcl/common/pca.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/filter.h>

#include "PointCloud.h"



int PalmPose::infer_palmpose(pcl::PointXYZ* v1, pcl::PointXYZ* v2, pcl::PointXYZ* v3, pcl::PointXYZ* p, int* size, pcl::PointXYZ* hand, pcl::PointXYZ* wrist)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(*size);
	pcl::PointXYZ* offset = p;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = p->x;
		cloud->points[i].y = p->y;
		cloud->points[i].z = p->z;
		p++;
	}

	pcl::PointXYZ handPoint(hand->x, hand->y, hand->z);
	pcl::PointXYZ wristPoint(wrist->x, wrist->y, wrist->z);
	float scale = pcl::geometry::distance(handPoint, wristPoint);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.03);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int cluster_index = -1;
	int max_cluster_size = 0;
	for (int i = 0;i < cluster_indices.size(); i++)
	{
		if (cluster_indices[i].indices.size() > max_cluster_size) {
			int cluster_size = cluster_indices[i].indices.size();
			max_cluster_size = cluster_size;
			cluster_index = i;
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
	if (cluster_index > -1)
	{
		auto& indices = cluster_indices[cluster_index].indices;
		for (int i = 0; i < indices.size(); i++)
		{
			cluster->points.push_back(cloud->points[indices[i]]);
		}
		
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;
	}


	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(-1.0f, wrist->z + scale * 0.25f);
	//pass.filter(*cloud_filtered);

	p = offset;
	for (size_t i = 0; i < cluster->points.size(); i++)
	{
		p->x = cluster->points[i].x;
		p->y = cluster->points[i].y;
		p->z = cluster->points[i].z;
		p++;
	}
	*size = cluster->points.size();
	if (*size < 100) {
		return 1;
	}


	//pcl::PCA<pcl::PointXYZ> cpca;
	//cpca.setInputCloud(cluster);
	//Eigen::Matrix3f mat = cpca.getEigenVectors();
	//Eigen::Vector3f v1_ = mat.col(0).normalized();
	//Eigen::Vector3f v2_ = mat.col(1).normalized();

	//Eigen::Vector3f handv(handPoint.x, handPoint.y, handPoint.z);
	//Eigen::Vector3f wristv(wristPoint.x, wristPoint.y, wristPoint.z);
	//Eigen::Vector3f pointing = handv - wristv;
	//if (pointing.dot(v1_) < 0.0f) {
	//	v1_ *= -1.0f;
	//}
	//if (pointing.dot(v2_) < 0.0f) {
	//	v2_ *= -1.0f;
	//}

	//Eigen::Vector3f v3_ = v1_.cross(v2_);

	//v1->x = v1_(0);
	//v1->y = v1_(1);
	//v1->z = v1_(2);
	//v2->x = v2_(0);
	//v2->y = v2_(1);
	//v2->z = v2_(2);
	//v3->x = v3_(0);
	//v3->y = v3_(1);
	//v3->z = v3_(2);

	return 0;
}

