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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  model (new pcl::PointCloud<pcl::PointXYZRGB>);

std::string listname_path, depth_path, pose_path, pc_path, model_path;
std::string depth_dir, save_dir, pose_dir, pc_dir, data_dir, conf_dir;

cv::Mat depth_img;
double fx, fy, cx, cy, depth_factor, clip_start, clip_end;
double z_max, z_min;
float max_r, max_l, crowd_ef, conf_min;
float lengthOBB[3];
int frame_count = 0;
int scale =10;

std::vector<float> translation, rot_quaternion;
std::vector<string> list_names, list_objects;

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
  double dz = (z_max - z_min);
  //point.r = uchar((abs(point.z - z_max) / dz) * 255); //upsidedown
  point.r = uchar((abs(point.z - z_min) / dz) * 255);
  point.g = point.r; point.b = point.r;                   
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

void depthToClould()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  scene (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointXYZRGB point;
   for(int row=0; row < depth_img.rows; row++)
    {
       for(int col=0; col < depth_img.cols; col++)       
        {
          if(isnan(depth_img.at<ushort>(row, col))) continue;
          double depth = clip_start + (clip_end - clip_start) * (depth_img.at<ushort>(row, col) / depth_factor);
          point.x = scale*(col-cx) * depth / fx;
          point.y = scale*(row-cy) * depth / fy;
          point.z = scale*depth;
          scene->push_back(point);
        }
    }
  Eigen::Vector3f b(translation[0], translation[1], translation[2]);
  Eigen::Quaternionf a(rot_quaternion[0], rot_quaternion[1], rot_quaternion[2], rot_quaternion[3]);
  pcl::transformPointCloud(*scene, *scene, b, a);

  scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int k=0; k < scene->size(); k++)
  {
    colorZ(scene->points[k]);
    if(scene->points[k].z > z_min)
      scene_cloud->push_back(scene->points[k]);
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

void read_save_poses()
{
  ifstream posefile (pose_path);
  string line;

  std::ofstream save_file;
  std::string pred_path = save_dir + list_names[frame_count] + ".txt";
  save_file.open(pred_path);

	Eigen::Vector3f row;
  Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity();
 
  std::string conf_path = conf_dir + list_names[frame_count] + ".txt";;
  ifstream pfile (conf_path);
  std::vector<float> conf_list;
  float conf_max = 0;

  if (pfile.is_open())                     
    {
      while (std::getline(pfile, line))                
      {
        float conf = std::stof(line);
        conf_list.push_back(conf);
        if(conf > conf_max) conf_max = conf;
      } 
    }

  int i=0;
  if (posefile.is_open())                     
    {
      while (std::getline(posefile, line))                
      {
        if(line == "") continue;
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
          int ins_id = (i+1)/4-1;
          float conf = conf_list[ins_id] / conf_max;
          std::cerr << conf_max << " " << conf_list[ins_id] << " " << conf << "\n";
          if(conf > conf_min) 
          {
              Eigen::Matrix3f rot_mat = pose_matrix.block<3,3>(0,0);
              Eigen::Vector3f ea = rot_mat.eulerAngles(0, 1, 2);
              Eigen::Matrix3f new_rot; 
              float rand_rot, rand_loc;

            for(int ix=0; ix<3; ix++)
            {
              int s = (-1)^(((i+1)%4)+ix);

              float max_loc = max_l*lengthOBB[0];

              rand_rot = static_cast <float> (rand()) / static_cast <float> (RAND_MAX / max_r);
              rand_loc = static_cast <float> (rand()) / static_cast <float> (RAND_MAX /max_loc);

              if(conf_list.size() < 15) 
              {
                
                rand_rot = rand_rot / (conf);
                rand_loc = rand_loc / (conf);
              }
              else if(conf_list.size() < 30)
              {
                rand_rot = crowd_ef*rand_rot / (conf);
                rand_loc = crowd_ef*rand_loc / (conf);
              }
              else 
              {
                rand_rot = crowd_ef*crowd_ef*rand_rot / (conf);
                rand_loc = crowd_ef*crowd_ef*rand_loc / (conf);
              }

              ea(ix) = ea(ix) + s * rand_rot;              
              pose_matrix(ix, 3) = pose_matrix(ix, 3)  + s * rand_loc;
            }
            new_rot = Eigen::AngleAxisf(ea(0), Eigen::Vector3f::UnitX())
                    * Eigen::AngleAxisf(ea(1), Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(ea(2), Eigen::Vector3f::UnitZ());
            pose_matrix.block<3,3>(0,0) = new_rot;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr  model_transf (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*model, *model_transf, pose_matrix);

            *model_cloud += *model_transf;
            for(int rr=0; rr < 3; rr++)
            {
              save_file << pose_matrix(rr, 0) << " " << pose_matrix(rr, 1) << " " << pose_matrix(rr, 2) << "\n"; 
            }
            save_file << row(0) << " " << row(1) << " " << row(2) << "\n";
          }
        }
        i++;
      }
    }
  save_file.close();
}

void read_poses()
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
          *model_cloud += *model_transf;
        }
        i++;
      }
    }
}

void compute_overlap()
{
  ifstream posefile (pose_path);
  string line;

  std::ofstream save_file;
  std::string conf_path = save_dir + list_names[frame_count] + ".txt";
  save_file.open(conf_path);

	Eigen::Vector3f row;
  Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity();
 
  ifstream pfile (pose_path);
  int num_line = 0;
  if (pfile.is_open())                     
    {
      while (std::getline(pfile, line))                
      {
        num_line++;
      }
    }
  //std::cerr << "num_line: " << num_line << "\n";

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
          *model_cloud += *model_transf;

          bool accept = false;
          float accept_r = lengthOBB[0];
          float accept_p = lengthOBB[0]/5.0;
          int np_pass = 0;
          for(int k=0; k < scene_cloud->size(); k++)
          {
            float delta_x = abs(scene_cloud->points[k].x - pose_matrix(0, 3));
            float delta_y = abs(scene_cloud->points[k].y - pose_matrix(1, 3)); 
            float delta_z = abs(scene_cloud->points[k].z - pose_matrix(2, 3)); 
            float dst = delta_x + delta_y + delta_z;
            if(delta_x > accept_r || delta_y > accept_r || delta_z > accept_r ) continue;

            for(int nk=0; nk < model_transf->size(); nk++)
            {              
              delta_x = abs(scene_cloud->points[k].x - model_transf->points[nk].x);
              delta_y = abs(scene_cloud->points[k].y - model_transf->points[nk].y); 
              delta_z = abs(scene_cloud->points[k].z - model_transf->points[nk].z); 
              dst = delta_x + delta_y + delta_z;
              if(dst < accept_p) 
              {
                np_pass = np_pass + 1;
                scene_cloud->points[k].r=255; scene_cloud->points[k].g=0; scene_cloud->points[k].b=0;
                nk = model_transf->size();
              }
            }
          }
          //std::cerr << "overlap: " << np_pass << "\n";
          if(i==3) save_file << np_pass;
          else save_file << "\n" << np_pass;
        }
        i++;
      }
    }
  save_file.close();
}

void get_listname()
{
  list_names.clear();
  ifstream name_file (listname_path);
  if (name_file.is_open())                     
    {
      while (!name_file.eof())                 
      {
        string name;
        getline (name_file, name);
        if(name != "")
          list_names.push_back(name);
        //std::cerr << "list_names: " << name << "\n"; 
      }
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_pose_visualization");
  ros::NodeHandle nh_, cloud_n, n;
  ros::Publisher model_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("model_cloud", 1);
  ros::Publisher scene_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("scene_cloud", 1);

  ros::Rate loop_rate(10);
  int method;

  nh_ = ros::NodeHandle("~");

  nh_.getParam("data_dir", data_dir);
  nh_.getParam("depth_dir", depth_dir);
  nh_.getParam("pose_dir", pose_dir);
  nh_.getParam("save_dir", save_dir);
  nh_.getParam("pc_dir", pc_dir);
  nh_.getParam("conf_dir", conf_dir);

  nh_.getParam("listname_path", listname_path);
  nh_.getParam("model_path", model_path);

  nh_.getParam("method", method);
  nh_.getParam("list_objects", list_objects); 

  nh_.getParam("fx", fx);
  nh_.getParam("fy", fy);
  nh_.getParam("cx", cx);
  nh_.getParam("cy", cy);
  nh_.getParam("clip_start", clip_start);
  nh_.getParam("clip_end", clip_end);
  nh_.getParam("depth_factor", depth_factor);

  nh_.getParam("camera_location", translation);
  nh_.getParam("camera_rot", rot_quaternion);

  nh_.getParam("max_r", max_r);
  nh_.getParam("max_l", max_l);
  nh_.getParam("crowd_ef", crowd_ef);
  nh_.getParam("conf_min", conf_min);

  nh_.getParam("z_max", z_max);
  nh_.getParam("z_min", z_min);

  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;

  std::cerr << listname_path << "\n";

  pcl::io::loadPLYFile<pcl::PointXYZRGB> (model_path, *model);
  OBB_Estimation();
  get_listname();
  int obi=0;

  while (ros::ok())
  {
      model_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

      if(method != 1)
      {
        pose_path = pose_dir + list_names[frame_count] + ".txt";
        pc_path = pc_dir + list_names[frame_count] + ".ply";
      }
      
      if(method==0) // depth to points and save pointcloud
      {
        depth_path = depth_dir + list_names[frame_count] + ".PNG";
        depth_img = cv::imread(depth_path, -1);
        //std::cerr << "depth image type: " << depth_img.type() << "\n";;
        depthToClould();
        //cv::imshow("depth image", depth_img);
        //cv::waitKey(3);
        read_poses();
        pcl::io::savePLYFileBinary (pc_path, *scene_cloud);
      }
      if(method==1) // compute overlap and save confidence score
      {
        if(frame_count==0) 
        {
          listname_path = data_dir + list_objects[obi] + "/list_names_full.txt";
          std::cerr << list_objects[obi] << "\n";
          get_listname();
          model_path = data_dir + list_objects[obi] + "/spare.ply";
          model.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::io::loadPLYFile<pcl::PointXYZRGB> (model_path, *model);
          OBB_Estimation();
          save_dir = data_dir + list_objects[obi] + "/conf-score/";
        }

        //std::cerr << list_names[frame_count] << "\n";

        pose_path = data_dir + list_objects[obi] + "/gt-txt/" + list_names[frame_count] + ".txt";
        pc_path = data_dir + list_objects[obi] + "/pointcloud/" + list_names[frame_count] + ".ply";

        loadClould();
        pcl::VoxelGrid<pcl::PointXYZRGB> avg;
        avg.setInputCloud(scene_cloud);
        avg.setLeafSize(0.03f, 0.03f, 0.03f);
      	scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        avg.filter(*scene_cloud);

        compute_overlap();
      }
      if(method==2) // add noise and save poses to gt-txt
      {
        loadClould();
        read_save_poses();
      }
      
      model_cloud->header.frame_id = "camera_depth_optical_frame";    
      scene_cloud->header.frame_id = "camera_depth_optical_frame";    

      pcl::toPCLPointCloud2(*model_cloud, cloud_filtered);
      pcl_conversions::fromPCL(cloud_filtered, output);
      model_pub.publish (output);

      pcl::toPCLPointCloud2(*scene_cloud, cloud_filtered);
      pcl_conversions::fromPCL(cloud_filtered, output);
      scene_pub.publish (output);
    
      ros::spinOnce();
      loop_rate.sleep();

      frame_count++;
      if(frame_count >= list_names.size())
      {
        frame_count=0;
        if(method == 1)
        {
          obi++;
          if(obi == list_objects.size()) return 0;
        }
        if(method == 2) return 0;
      }
  }

  return 0;
}