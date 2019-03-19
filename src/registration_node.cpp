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

void remove_overlap(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, double thresh)
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

	double dist;
	for (int i = 0; i < source.size(); ++i)
	{
		if (kdtree.nearestKSearch(source_cloud->points[i], 3, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			/* pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int k = 0; k < 3; k++)
			{
				plane_cloud->push_back(target_cloud->points[pointIdxNKNSearch[k]]);
			}
			double a, b, c, d;
			if (threePointToPlane(*plane_cloud, a, b, c, d))
			{
				dist = pcl::pointToPlaneDistance(source.points[i], a, b, c, d);
			}
			else */ dist = sqrt(pointNKNSquaredDistance[0]);
      		if(dist > thresh) target.push_back(source.points[i]);
		}
	}

	return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OBB_Drawer");
  ros::NodeHandle nh_, cloud_n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("registed_cloud", 1);
  ros::Rate loop_rate(10);

  std::string source_path, target_path, saved_path; 
  double neighbor_dist;
  std::vector<float> translation;
  std::vector<float> rot_quaternion;

  nh_ = ros::NodeHandle("~");
  nh_.getParam("source_path", source_path);
  nh_.getParam("target_path", target_path);
  nh_.getParam("saved_path", saved_path);
  nh_.getParam("neighbor_dist", neighbor_dist);
  nh_.getParam("translation", translation);
  nh_.getParam("rot_quaternion", rot_quaternion);

  pcl::io::loadPLYFile<pcl::PointXYZRGB> (source_path, *source_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (target_path, *target_cloud);

  Eigen::Vector3f b(translation[0], translation[1], translation[2]);
  Eigen::Quaternionf a(rot_quaternion[0], rot_quaternion[1], rot_quaternion[2], rot_quaternion[3]);
  pcl::transformPointCloud(*source_cloud, *source_cloud, b, a);

  remove_overlap(*source_cloud, *target_cloud, neighbor_dist);
  //*target_cloud += *source_cloud;

  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;
  target_cloud->header.frame_id = "camera_depth_optical_frame";
  pcl::io::savePLYFileBinary (saved_path, *target_cloud);  
  pcl::toPCLPointCloud2(*target_cloud, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  
  while (ros::ok())
  {  
    cloud_pub.publish (output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}