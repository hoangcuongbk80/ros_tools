#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
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
#include <pcl/registration/icp.h>

//OpenCV
// OpenCV specific includes
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

// Boost
#include <boost/math/special_functions/round.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>

using namespace std;
using namespace cv;
using namespace Eigen;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  model (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  scene_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gt_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  baseline_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  ours_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

std::string pc_path, gt_path, baseline_path, ours_path, model_path, listname_path;
std::string pc_dir, gt_dir, baseline_dir, ours_dir;

float z_max, z_min;
int scale =10;
int frame_count = 0;
float lengthOBB[3], error_max, ratio_error_max;

std::vector<float> translation, rot_quaternion;
std::vector<string> list_names;

void colorZ(pcl::PointXYZRGB &point)
{
  if(point.z > z_max)
  {
    point.r = 255; point.g = 255; point.b = 255;
    return;
  }
  if(point.z < z_min)
  {
    point.r = 0; point.g = 0; point.b = 0;
    return;
  }
  float dz = (z_max - z_min);
  //point.r = uchar((abs(point.z - z_max) / dz) * 255); //upsidedown
  point.r = uchar((abs(point.z - z_min) / dz) * 255);
  point.g = point.r; point.b = point.r;                   
}

void color_dst(float dist, float min, float max, float &r, float &g, float &b)
{
	b = 0;
	float mid = (max+min) / 2.0;
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

void ADD_metric(pcl::PointCloud<pcl::PointXYZRGB> &pred_model, Eigen::Matrix4f pred_pose, Eigen::Matrix4f gt_pose, float error_max)
{
  pred_model.clear();

  std::cerr << "pred_pose: " << pred_pose << "\n";
  std::cerr << "gt_pose: " << gt_pose << "\n";

	for (int i = 0; i < model->size(); ++i)
	{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pred (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gt (new pcl::PointCloud<pcl::PointXYZRGB>);
    pred->push_back(model->points[i]);
    gt->push_back(model->points[i]);
    pcl::transformPointCloud(*pred, *pred, pred_pose);
    pcl::transformPointCloud(*gt, *gt, gt_pose);

    float dist_x = abs(pred->points[0].x - gt->points[0].x);
    float dist_y = abs(pred->points[0].y - gt->points[0].y);
    float dist_z = abs(pred->points[0].z - gt->points[0].z);
		float dst = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);

    float c_r, c_g, c_b;
    color_dst(dst, 0, error_max, c_r, c_g, c_b);
    pred->points[0].r = c_r;
    pred->points[0].g = c_g;
    pred->points[0].b = c_b;
    pred_model.push_back(pred->points[0]);
	}
}

void SYM_metric(pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, float error_max)
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

	float dist;
	float Max_dist = -1 * DBL_MAX;
	float Min_dist = DBL_MAX;
	for (int i = 0; i < source.size(); ++i)
	{
		if (kdtree.nearestKSearch(source_cloud->points[i], 3, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int k = 0; k < 3; k++)
			{
				plane_cloud->push_back(target_cloud->points[pointIdxNKNSearch[k]]);
			}
			float a, b, c, d;
			dist = sqrt(pointNKNSquaredDistance[0]);

			float c_r, c_g, c_b;
			color_dst(dist, 0, error_max, c_r, c_g, c_b);
			source.points[i].r = c_r;
			source.points[i].g = c_g;
			source.points[i].b = c_b;
		}
	}
}

int OBB_Estimation()
{
    if(!model->size())
    {
      std::cerr << "cloud is empty!" << "\n";
      return 0;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr OBB (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZRGB point;
    
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*model, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*model, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*model, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    
    lengthOBB[0] = fabs(maxPoint.x - minPoint.x); //MAX length OBB
    lengthOBB[1] = fabs(maxPoint.y - minPoint.y); //MID length OBB
    lengthOBB[2] = fabs(maxPoint.z - minPoint.z); //MIN length OBB
    if(lengthOBB[1] > lengthOBB[0]) lengthOBB[0] = lengthOBB[1];
    if(lengthOBB[2] > lengthOBB[0]) lengthOBB[0] = lengthOBB[2];
}

void loadClould()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  scene (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (pc_path, *scene);
  for(int k=0; k < scene->size(); k++)
  {
    colorZ(scene->points[k]);
    if(scene->points[k].z > z_min)
      scene_cloud->push_back(scene->points[k]);
  }
}

void compute_error(pcl::PointCloud<pcl::PointXYZRGB> &pred_model, Eigen::Matrix4f pred_pose)
{
  ifstream posefile (gt_path);
  string line;

	Eigen::Vector3f row;
  Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f gt_pose = Eigen::Matrix4f::Identity();
 
  int i=0;
  float min_dst = 10000;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  target_model (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (posefile.is_open())                     
    {
      while (std::getline(posefile, line))                
      {
        vector<string> st;
        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
        row(0) = std::stof(st[0]); row(1) = std::stof(st[1]); row(2) = std::stof(st[2]); //translaton
        int j=(i+1)%4;
        if(j==0)
        {
          pose_matrix(0, 3) = scale*row(0);
          pose_matrix(1, 3) = scale*row(1);
          pose_matrix(2, 3) = scale*row(2);
        }
        else
        {
          pose_matrix(j-1, 0) = row(0);
          pose_matrix(j-1, 1) = row(1);
          pose_matrix(j-1, 2) = row(2);
        }
        if(j==0)
        {
          float dx = abs(pred_pose(0,3)-pose_matrix(0,3));
          float dy = abs(pred_pose(1,3)-pose_matrix(1,3));
          float dz = abs(pred_pose(2,3)-pose_matrix(2,3));
          float dst = sqrt(dx*dx+dy*dy+dz*dz);
          if(dst < min_dst)
          {
            min_dst = dst;
            pcl::transformPointCloud(*model, *target_model, pose_matrix);
            //gt_pose.block<4,4>(0,0) = pose_matrix.block<4,4>(0,0);
            gt_pose = pose_matrix.replicate(1,1);
          }
        }
        i++;
      }
    }
  
  // SYM_metric(pred_model, *target_model, error_max);
  ADD_metric(pred_model, pred_pose, gt_pose, error_max);
}

void read_poses(std::string pose_path, pcl::PointCloud<pcl::PointXYZRGB> &output, bool error_est)
{
  ifstream posefile (pose_path);
  string line;

	Eigen::Vector3f row;
  Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity();
 
  int i=0;
  if (posefile.is_open())                     
    {
      while (std::getline(posefile, line))                
      {
        vector<string> st;
        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
        row(0) = std::stof(st[0]); row(1) = std::stof(st[1]); row(2) = std::stof(st[2]); //translaton
        int j=(i+1)%4;
        if(j==0)
        {
          pose_matrix(0, 3) = scale*row(0);
          pose_matrix(1, 3) = scale*row(1);
          pose_matrix(2, 3) = scale*row(2);
        }
        else
        {
          pose_matrix(j-1, 0) = row(0);
          pose_matrix(j-1, 1) = row(1);
          pose_matrix(j-1, 2) = row(2);
        }
        if(j==0)
        {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr  model_transf (new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::transformPointCloud(*model, *model_transf, pose_matrix);
          if(error_est) compute_error(*model_transf, pose_matrix);
          output += *model_transf;
        }
        i++;
      }
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimation_evaluation");
  ros::NodeHandle nh_, cloud_n, n;
  ros::Publisher gt_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("gt_cloud", 1);
  ros::Publisher baseline_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("baseline_cloud", 1);
  ros::Publisher ours_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("ours_cloud", 1);
  ros::Publisher scene_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("scene_cloud", 1);

  ros::Rate loop_rate(10);

  nh_ = ros::NodeHandle("~");
  
  nh_.getParam("baseline_dir", baseline_dir);
  nh_.getParam("proposed_dir", ours_dir);
  nh_.getParam("gt_dir", gt_dir);
  nh_.getParam("pc_dir", pc_dir);

  nh_.getParam("listname_path", listname_path);
  nh_.getParam("model_path", model_path);
  nh_.getParam("z_max", z_max);
  nh_.getParam("z_min", z_min);
  nh_.getParam("ratio_error_max", ratio_error_max);


  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;

  std::cerr << listname_path << "\n";

  pcl::io::loadPLYFile<pcl::PointXYZRGB> (model_path, *model);
  OBB_Estimation();
  error_max = lengthOBB[0] * ratio_error_max;

  ifstream name_file (listname_path);
  if (name_file.is_open())                     
    {
      while (!name_file.eof())                 
      {
        string name;
        getline (name_file, name);
        list_names.push_back(name);
        std::cerr << "list_names: " << name << "\n"; 
      }
    }

  while (ros::ok())
  {
      if(list_names[frame_count] == "") 
      {
        frame_count=0;
        std::cerr << "Finished!" << "\n";
      }
      scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      gt_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      baseline_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      ours_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

      pc_path = pc_dir + list_names[frame_count] + ".ply";
      gt_path = gt_dir + list_names[frame_count] + ".txt";
      baseline_path = baseline_dir + list_names[frame_count] + ".txt";
      ours_path = ours_dir + list_names[frame_count] + ".txt";
      
      loadClould();

      read_poses(gt_path, *gt_cloud, false);
      for(int k=0; k < gt_cloud->size(); k++)
        colorZ(gt_cloud->points[k]);

      read_poses(baseline_path, *baseline_cloud, true);
      read_poses(ours_path, *ours_cloud, true);

      gt_cloud->header.frame_id = "camera_depth_optical_frame";    
      baseline_cloud->header.frame_id = "camera_depth_optical_frame";    
      ours_cloud->header.frame_id = "camera_depth_optical_frame";    
      scene_cloud->header.frame_id = "camera_depth_optical_frame";    

      pcl::toPCLPointCloud2(*gt_cloud, cloud_filtered);
      pcl_conversions::fromPCL(cloud_filtered, output);
      gt_pub.publish (output);

      pcl::toPCLPointCloud2(*baseline_cloud, cloud_filtered);
      pcl_conversions::fromPCL(cloud_filtered, output);
      baseline_pub.publish (output);

      pcl::toPCLPointCloud2(*ours_cloud, cloud_filtered);
      pcl_conversions::fromPCL(cloud_filtered, output);
      ours_pub.publish (output);

      pcl::toPCLPointCloud2(*scene_cloud, cloud_filtered);
      pcl_conversions::fromPCL(cloud_filtered, output);
      scene_pub.publish (output);
    
      ros::spinOnce();
      loop_rate.sleep();

      frame_count++;
      if(frame_count >= list_names.size())
      {
        frame_count=0;
      }
  }

  return 0;
}