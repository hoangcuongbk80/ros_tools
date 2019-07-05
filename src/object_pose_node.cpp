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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  scene_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pub_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

std::string depth_path, rgb_path, pose_path;
cv::Mat rgb_img, depth_img;
double fx, fy, cx, cy, depth_factor;

std::vector<string> model_paths;
std::vector<string> classes;
std::vector<Eigen::Matrix4f> transforms;

void depthToClould()
{
  depth_img = cv::imread(depth_path, -1);
  rgb_img = cv::imread(rgb_path, -1);

   scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointXYZRGB point;
   for(int row=0; row < depth_img.rows; row++)
    {
       for(int col=0; col < depth_img.cols; col++)       
        {
          if(isnan(depth_img.at<ushort>(row, col))) continue;
          double depth = depth_img.at<ushort>(row, col) / depth_factor;
          point.x = (col-cx) * depth / fx;
          point.y = (row-cy) * depth / fy;
          point.z = depth;
          point.b = rgb_img.at<cv::Vec3b>(row, col)[0];
          point.g = rgb_img.at<cv::Vec3b>(row, col)[1];
          point.r = rgb_img.at<cv::Vec3b>(row, col)[2];
         scene_cloud->push_back(point);
        }
    }
}

void colorMap(int i, pcl::PointXYZRGB &point)
{
  if (i == 1) // red  
  {
    point.r = 255; point.g = 0; point.b = 0; 
  }
  else if (i == 2) //line
  {
    point.r = 0; point.g = 255; point.b = 0;
  }
  else if ( i == 3) //blue
  { 
    point.r = 0; point.g = 0; point.b = 255;
  } 
  else if ( i == 4) //maroon
  {
    point.r = 128; point.g = 0; point.b = 0;

  }
  else if ( i == 5) //green
  {
    point.r = 0; point.g = 128; point.b = 0;
  }  
  else if ( i == 6) //navy
  {
    point.r = 0; point.g = 0; point.b = 128;
  }
  else if ( i == 7) //yellow
  {
    point.r = 255; point.g = 255; point.b = 0;
  }
  else if ( i == 8) //magenta
  {
    point.r = 255; point.g = 0; point.b = 255;
  }
  else if ( i == 9) //cyan
  {
    point.r = 0; point.g = 255; point.b = 255;
  }    
  else if ( i == 10) //olive
  {
    point.r = 128; point.g = 128; point.b = 0;
  }
  else if ( i == 11) //purple
  {
    point.r = 128; point.g = 0; point.b = 128;
  } 
    
  else if ( i == 12) //teal
  {
    point.r = 0; point.g = 128; point.b = 128;
  }
    
  else if ( i == 13) 
  {
    point.r = 92; point.g = 112; point.b = 92;
  }
  else if ( i == 14) //brown
  {
    point.r = 165; point.g = 42; point.b = 42;
  }    
  else //silver
  {
    point.r = 192; point.g = 192; point.b = 192;
  }                   
}

void loadModels()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr  model_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  color_model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  for(int i=0; i < model_paths.size(); i++)
  {
    pcl::io::loadPLYFile<pcl::PointXYZ> (model_paths[i], *model_cloud);
    pcl::transformPointCloud(*model_cloud, *model_cloud, transforms[i]);
    copyPointCloud(*model_cloud, *color_model_cloud);
    for(int k=0; k < color_model_cloud->size(); k++)
    {
      colorMap(i, color_model_cloud->points[k]);
    }
    *pub_cloud += *color_model_cloud;
  }
}

void extract_transform_from_quaternion(std::string line, Eigen::Matrix4f &T, int class_index)
{
	  Eigen::Vector3f trans;
    float rot_quaternion[4];
    vector<string> st;
    boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
    trans(0) = std::stof(st[4]); trans(1) = std::stof(st[5]); trans(2) = std::stof(st[6]); //translaton
    rot_quaternion[0] = std::stof(st[0]); rot_quaternion[1] = std::stof(st[1]); //rotation
    rot_quaternion[2] = std::stof(st[2]); rot_quaternion[3] = std::stof(st[3]); //rotation

    Eigen::Quaternionf q(rot_quaternion[0], rot_quaternion[1], rot_quaternion[2], rot_quaternion[3]); //w x y z
    
    T.block(0, 3, 3, 1) = trans;
    T.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_pose_visualization");
  ros::NodeHandle nh_, cloud_n, n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("my_cloud", 1);
  ros::Rate loop_rate(10);

  std::string data_dir, model_dir, classes_path; 
  int num_frames;

  nh_ = ros::NodeHandle("~");
  nh_.getParam("data_dir", data_dir);
  nh_.getParam("model_dir", model_dir);
  nh_.getParam("classes_path", classes_path);
  nh_.getParam("pose_path", pose_path);
  nh_.getParam("num_frames", num_frames);  
  nh_.getParam("fx", fx);
  nh_.getParam("fy", fy);
  nh_.getParam("cx", cx);
  nh_.getParam("cy", cy);
  nh_.getParam("depth_factor", depth_factor); 

  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;

  std::cerr << "pose_path: " << pose_path << "\n";

  ifstream classes_file (classes_path);
  if (classes_file.is_open())                     
    {
      while (!classes_file.eof())                 
      {
        string cls;
        getline (classes_file, cls);
        classes.push_back(cls);
      }
    }
    else 
    {
      std::cerr << "Unable to open " << classes_path  << " file" << "\n";
      exit(0);
    }
  
  ifstream posefile (pose_path);    
  string line;
  bool firstLine = true;                             
  
  while (ros::ok())
  {
    pub_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    model_paths.clear();
    transforms.clear();

    if (posefile.is_open())                     
    {
      if (!posefile.eof())                 
      {
        if(firstLine) 
        {
          getline (posefile, line);
          firstLine = false;
        }
        depth_path = data_dir + line + "-depth.png";
        rgb_path = data_dir + line + "-color.png";            
        line = "hi" ;
        std::cerr << rgb_path << "\n";
        while(true)
        {
          getline (posefile, line);
          std::cerr << line <<  endl;          
          if(line.length() == 6 | line=="") break;
          int cls_index = std::stoi(line) - 1;
          std::string model_path = model_dir + classes[cls_index] + "/points.ply";
          model_paths.push_back(model_path);
          getline (posefile, line);
          Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
          extract_transform_from_quaternion(line, T, cls_index);          
          transforms.push_back(T);
        }
      }
    }
    else 
    {
      std::cerr << "Unable to open file";
      exit(0);
    }
  
    loadModels();
    depthToClould();
    *pub_cloud += *scene_cloud;
    pub_cloud->header.frame_id = "camera_depth_optical_frame";    

    pcl::toPCLPointCloud2(*pub_cloud, cloud_filtered);
    pcl_conversions::fromPCL(cloud_filtered, output);
    cloud_pub.publish (output);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  posefile.close();
  return 0;
}