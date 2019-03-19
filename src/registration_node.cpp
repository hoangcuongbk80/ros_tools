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

void remove_overlap(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, double thresh, int num_neighbors)
{
	if (source.size() == 0 || target.size() == 0) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(source, *source_cloud);
	pcl::copyPointCloud(target, *target_cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);
	std::vector<int> pointIdxNKNSearch(num_neighbors);
	std::vector<float> pointNKNSquaredDistance(num_neighbors);

	for (int i = 0; i < source.size(); ++i)
	{
		double dist = 0;;
		if (kdtree.nearestKSearch(source_cloud->points[i], 3, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for(int j=0; j < num_neighbors; j++)
			{
				dist += sqrt(pointNKNSquaredDistance[j]);
			}
			dist = (double) dist / num_neighbors;
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
  int num_neighbors;
  std::vector<float> translation;
  std::vector<float> rot_quaternion;

  nh_ = ros::NodeHandle("~");
  nh_.getParam("source_path", source_path);
  nh_.getParam("target_path", target_path);
  nh_.getParam("saved_path", saved_path);
  nh_.getParam("neighbor_dist", neighbor_dist);
  nh_.getParam("num_neighbors", num_neighbors);
  nh_.getParam("translation", translation);
  nh_.getParam("rot_quaternion", rot_quaternion);

  pcl::io::loadPLYFile<pcl::PointXYZRGB> (source_path, *source_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (target_path, *target_cloud);

  Eigen::Vector3f b(translation[0], translation[1], translation[2]);
  Eigen::Quaternionf a(rot_quaternion[0], rot_quaternion[1], rot_quaternion[2], rot_quaternion[3]);
  pcl::transformPointCloud(*source_cloud, *source_cloud, b, a);

  remove_overlap(*source_cloud, *target_cloud, neighbor_dist, num_neighbors);
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