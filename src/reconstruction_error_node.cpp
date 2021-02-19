#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/geometry.h>

#include <cmath>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr  error_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

void colorMap(double dist, double min, double max, double &r, double &g, double &b)
{
	b = 0;
	double mid = (max+min) / 2.0;
	if(dist < mid & dist > min)
	{
		g = 255;
		r = (dist/mid)*255;
	}
	else if(dist < max & dist > min)
	{
		r = 255;
		g = (max-dist)*255;
	}
	else
	{
		r = 255; g = 0; b = 0; 
	}
}

bool threePointToPlane(pcl::PointCloud<pcl::PointXYZ> &input, double &a, double &b, double &c, double &d)
{
	if (input.size() < 3) return false;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(input, *cloud);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.1);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) return false;
	else
	{
		a = coefficients->values[0];
		b = coefficients->values[1];
		c = coefficients->values[2];
		d = coefficients->values[3];
		return true;
	}
}

void Distances_CloudToCloud_PCL(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target,
	pcl::PointCloud<pcl::PointXYZ> &dists_Cloud, double error_max)
{
	if (source.size() == 0 || target.size() == 0) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(source, *source_cloud);
	pcl::copyPointCloud(target, *target_cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);
	std::vector<int> pointIdxNKNSearch(3);
	std::vector<float> pointNKNSquaredDistance(3);

	double meandist = 0;
	double dist;
	double Max_dist = -1 * DBL_MAX;
	double Min_dist = DBL_MAX;
	for (int i = 0; i < source.size(); ++i)
	{
		if (kdtree.nearestKSearch(source_cloud->points[i], 3, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int k = 0; k < 3; k++)
			{
				plane_cloud->push_back(target_cloud->points[pointIdxNKNSearch[k]]);
			}
			double a, b, c, d;
			if (threePointToPlane(*plane_cloud, a, b, c, d))
			{
				dist = pcl::pointToPlaneDistance(source.points[i], a, b, c, d);
			}
			else dist = sqrt(pointNKNSquaredDistance[0]);

			double c_r, c_g, c_b;
			colorMap(dist, 0, error_max, c_r, c_g, c_b);
			source.points[i].r = c_r;
			source.points[i].g = c_g;
			source.points[i].b = c_b;

			meandist += dist;
			pcl::PointXYZ point;
			point.x = dist;
			point.y = 0;
			point.z = 0;
			dists_Cloud.push_back(point);
			if (Max_dist < dist) Max_dist = dist;
			if (Min_dist > dist) Min_dist = dist;
		}
	}

	//calculating the mean distance
	meandist = (double)meandist / source.size();
	pcl::PointXYZ point;
	point.x = meandist;
	point.y = Max_dist;
	point.z = Min_dist;
	dists_Cloud.push_back(point);
	std::cerr << "dist max min mean: " << point.x << "	" << point.y << "	" << point.z << "\n";
	return;
}

void loadBackGround(std::string filename)
{
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (filename, *source_cloud);
  
  for(int k=0; k<source_cloud->size(); k++)
  {
    pcl::PointXYZRGB point;
    point.x = source_cloud->points[k].x * 1000; //mm
    point.y = source_cloud->points[k].y * 1000;
    point.z = source_cloud->points[k].z * 1000;
    point.r=0; point.g=0; point.b=255;
    myCloud->push_back(point);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Reconstruction_Error_Heat_Map");
  ros::NodeHandle nh_, cloud_n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("error_cloud", 1);
  ros::Rate loop_rate(10);

  std::string source_dir, target_dir;
  double error_max, X_Rot, X_Rot_Inv, Y_Rot, Y_Rot_Inv, Z_Rot, Z_Rot_Inv;
  nh_ = ros::NodeHandle("~");
  nh_.getParam("source_dir", source_dir);
  nh_.getParam("target_dir", target_dir);
  nh_.getParam("error_max", error_max);
  nh_.getParam("X_Rot", X_Rot);
  nh_.getParam("Y_Rot", Y_Rot);
  nh_.getParam("Z_Rot", Z_Rot);
  nh_.getParam("X_Rot_Inv", X_Rot_Inv);
  nh_.getParam("Y_Rot_Inv", Y_Rot_Inv);
  nh_.getParam("Z_Rot_Inv", Z_Rot_Inv);

  std::cerr << "error_max X_Rot X_Rot_Inv: " << error_max << " " << X_Rot << " " << X_Rot_Inv << "\n";  


  loadBackGround(target_dir);
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (source_dir, *source_cloud);
  for(int k=0; k<source_cloud->size(); k++)
  {
    source_cloud->points[k].x = source_cloud->points[k].x * 1000;
    source_cloud->points[k].y = source_cloud->points[k].y * 1000;
    source_cloud->points[k].z = source_cloud->points[k].z * 1000;
  }

  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  //transform_2.translation() << 0.1, 0.0, 0.1;
  transform_1.rotate (Eigen::AngleAxisf (X_Rot, Eigen::Vector3f::UnitX()));
  transform_1.rotate (Eigen::AngleAxisf (Y_Rot, Eigen::Vector3f::UnitY()));
  transform_1.rotate (Eigen::AngleAxisf (Z_Rot, Eigen::Vector3f::UnitZ()));

  transform_2.rotate (Eigen::AngleAxisf (Z_Rot_Inv, Eigen::Vector3f::UnitZ()));
  transform_2.rotate (Eigen::AngleAxisf (Y_Rot_Inv, Eigen::Vector3f::UnitY()));
  transform_2.rotate (Eigen::AngleAxisf (X_Rot_Inv, Eigen::Vector3f::UnitX()));
  
  pcl::transformPointCloud (*source_cloud, *target_cloud, transform_1);
  pcl::transformPointCloud (*target_cloud, *target_cloud, transform_2);

  Distances_CloudToCloud_PCL(*source_cloud, *target_cloud, *error_cloud, error_max);
  *myCloud += *source_cloud;

  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;
  myCloud->header.frame_id = "camera_depth_optical_frame";
  pcl::toPCLPointCloud2(*myCloud, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  
  while (ros::ok())
  {  
    cloud_pub.publish (output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}