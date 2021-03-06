#include "stdafx.h"

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

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
	depth_limit = wrist->z + scale * 0.1f;

	// pass through filter
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.0f, depth_limit);
	pass.filter(*cloud);

	if (cloud->points.size() < min_point_size)
	{
		return -1;
	}

	// optional? euclidean cluster 
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
	
	if (cluster->points.size() < min_point_size) {
		return -1;
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

	float A = coefficients->values[0];
	float B = coefficients->values[1];
	float C = coefficients->values[2];
	Vector3f vn(A, B, C);

	// Vx is the projection of principal component onto the fit plane.
	Vector3f Vx = principal - vn * principal.dot(vn) / vn.squaredNorm();
	Vx.normalize();
	Vector3f handv(handPoint.x, handPoint.y, handPoint.z);
	Vector3f wristv(wristPoint.x, wristPoint.y, wristPoint.z);
	Vector3f pointing = handv - wristv;
	if (pointing.dot(Vx) < 0.0f) {
		Vx *= -1.0f;
	}

	// Create a Concave Hull representation of the projected inliers
	// we consider this as the contour of the hand
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cluster);
	//chull.setAlpha(0.1);
	chull.reconstruct(*cloud_hull);

	// find out the palm direction
	//Vector4f line_pt(handPoint.x, handPoint.y, handPoint.z, 1.0f);
	pcl::PointXYZ centroid;
	pcl::computeCentroid(*cloud_hull, centroid);
	Vector4f line_pt(centroid.x, centroid.y, centroid.z, 1.0f);
	Vector4f line_dir(Vx.x(), Vx.y(), Vx.z(), 0.0f);
	float current_max_sqr_length = -1.0f;
	Vector4f far_pt;
	for (int i = 0; i < cloud_hull->points.size(); i++)
	{
		Vector4f pt(cloud_hull->points[i].x, cloud_hull->points[i].y, cloud_hull->points[i].z, 1.0f);
		float distance = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);
		if (distance > current_max_sqr_length)
		{
			current_max_sqr_length = distance;
			far_pt = pt;
		}
	}

	Vector4f thumb_dir4f = far_pt - line_pt;
	Vector3f thumb_dir(thumb_dir4f.x(), thumb_dir4f.y(), thumb_dir4f.z());

	Vector3f Vz = vn.normalized();
	Vector3f Vy = Vz.cross(Vx).normalized();
	if (Vy.dot(thumb_dir) > 0.0f) {
		Vy *= -1.0f;
		Vz *= -1.0f;
	}
	// It is the improper rotation
	// cross product again
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

	return 0;
}
