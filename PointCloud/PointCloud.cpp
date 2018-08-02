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

using namespace Eigen;

int PalmPose::infer_palmpose(pcl::PointXYZ* v1, pcl::PointXYZ* v2, pcl::PointXYZ* v3, pcl::PointXYZ* p, int* size, pcl::PointXYZ* hand, pcl::PointXYZ* wrist)
{
	const int min_point_size = 100;
	const float cluster_tolerance = 0.02f;
	int min_cluster_size = 120;
	int max_cluster_size = 40000;
	float depth_limit = 0.0f;

	// copy the points
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

	// use a depth estimation
	pcl::PointXYZ handPoint(hand->x, hand->y, hand->z);
	pcl::PointXYZ wristPoint(wrist->x, wrist->y, wrist->z);
	float scale = pcl::geometry::distance(handPoint, wristPoint);
	depth_limit = wrist->z + scale * 0.3f;

	// pass through filter
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.0f, depth_limit);
	pass.filter(*cloud);

	if (cloud->points.size() < min_point_size)
	{
		return 1;
	}


	// euclidean cluster (It might take longer)
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(cluster_tolerance);
	ec.setMinClusterSize(min_cluster_size);
	ec.setMaxClusterSize(max_cluster_size);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int cluster_index = -1;
	int current_cluster_size = 0;
	for (int i = 0;i < cluster_indices.size(); i++)
	{
		if (cluster_indices[i].indices.size() > current_cluster_size) {
			int cluster_size = cluster_indices[i].indices.size();
			current_cluster_size = cluster_size;
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

	// optional: fill the output 
	p = offset;
	for (size_t i = 0; i < cluster->points.size(); i++)
	{
		p->x = cluster->points[i].x;
		p->y = cluster->points[i].y;
		p->z = cluster->points[i].z;
		p++;
	}
	*size = cluster->points.size();

	if (*size < min_point_size) {
		return 1;
	}

	// find the principal component
	pcl::PCA<pcl::PointXYZ> cpca;
	cpca.setInputCloud(cluster);
	Matrix3f mat = cpca.getEigenVectors();
	Vector3f principal = mat.col(0).normalized();
	
	// fit the plane
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cluster);
	seg.segment(*inliers, *coefficients);

	static Vector3f last_vn(0, 0, 1.0f);

	float A = coefficients->values[0];
	float B = coefficients->values[1];
	float C = coefficients->values[2];
	Vector3f vn(A, B, C);
	if (vn.dot(last_vn) < 0.0f) {
		vn *= -1.0f;
	}
	last_vn = vn;
	// Vx is the projection of principal component onto the fit plane.
	Vector3f Vx = principal - vn * principal.dot(vn) / vn.squaredNorm();
	Vx.normalize();
	Vector3f handv(handPoint.x, handPoint.y, handPoint.z);
	Vector3f wristv(wristPoint.x, wristPoint.y, wristPoint.z);
	Vector3f pointing = handv - wristv;
	if (pointing.dot(Vx) > 0.0f) {
		Vx *= -1.0f;
	}

	Vector3f Vz = vn.normalized();
	Vector3f Vy = Vx.cross(Vz).normalized();
	Vx = Vy.cross(Vz).normalized();

	static Quaternionf last_q_rot(0, 1.0f, 1.0f, 1.0f);
	Matrix3f rot;
	rot.col(0) = Vx;
	rot.col(1) = Vy;
	rot.col(2) = Vz;

	Quaternionf q_rot(rot);
	Quaternionf q_result = q_rot.slerp(0.5f, last_q_rot).normalized();
	last_q_rot = q_result;

	Matrix3f mat_rot = q_result.toRotationMatrix();
	v1->x = mat_rot(0, 0);
	v1->y = mat_rot(1, 0);
	v1->z = mat_rot(2, 0);
	v2->x = mat_rot(0, 1);
	v2->y = mat_rot(1, 1);
	v2->z = mat_rot(2, 1);
	v3->x = mat_rot(0, 2);
	v3->y = mat_rot(1, 2);
	v3->z = mat_rot(2, 2);

	return 0;
}
