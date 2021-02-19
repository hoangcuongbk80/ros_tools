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

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

bool use_Kalman_Filter = false;
Vector6f est_pose[11];
Matrix6f est_cov[11];
bool first_observe[11];
int frame_count;
Eigen::Matrix4f cam_T;

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
  pcl::transformPointCloud(*scene_cloud, *scene_cloud, cam_T);
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
      colorMap(i+1, color_model_cloud->points[k]);
    }
    *pub_cloud += *color_model_cloud;
  }
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  depth_img = cv::imread(depth_path, -1);
  //std::cerr << "depth image type: " << depth_img.type();
  rgb_img = cv::imread(rgb_path, -1);
  depthToClould();
  //cv::imshow("RGB image", rgb_img);
  //cv::waitKey(3);
  //cv::imshow("depth image", depth_img);
  //cv::waitKey(3);
  //std::cerr << rgb_path << "\n";
  //std::cerr << depth_path << "\n";
}

inline Vector6f subPose(const Vector6f &origin, const Vector6f &pose)
{
    Eigen::Affine3f origin_A = Eigen::Affine3f::Identity();
    origin_A.translation() << origin(0), origin(1), origin(2);
    origin_A.rotate (Eigen::AngleAxisf (origin(3), Eigen::Vector3f::UnitX()));
    origin_A.rotate (Eigen::AngleAxisf (origin(4), Eigen::Vector3f::UnitY()));
    origin_A.rotate (Eigen::AngleAxisf (origin(5), Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f pose_A = Eigen::Affine3f::Identity();
    pose_A.translation() << pose(0), pose(1), pose(2);
    pose_A.rotate (Eigen::AngleAxisf (origin(3), Eigen::Vector3f::UnitX()));
    pose_A.rotate (Eigen::AngleAxisf (origin(4), Eigen::Vector3f::UnitY()));
    pose_A.rotate (Eigen::AngleAxisf (origin(5), Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f result_A;
    result_A.matrix() = origin_A.matrix()*pose_A.matrix().inverse();
    Vector3f rot = result_A.rotation().eulerAngles(0, 1, 2);

    Vector6f result;
    result(0) = result_A.translation()(0); result(1) = result_A.translation()(1); result(2) = result_A.translation()(2);
    result(3) = rot(0); result(4) = rot(1); result(5) = rot(3);
    return result;
}

inline Vector6f addPose(const Vector6f &origin, const Vector6f &pose)
{
    Eigen::Affine3f origin_A = Eigen::Affine3f::Identity();
    origin_A.translation() << origin(0), origin(1), origin(2);
    origin_A.rotate (Eigen::AngleAxisf (origin(3), Eigen::Vector3f::UnitX()));
    origin_A.rotate (Eigen::AngleAxisf (origin(4), Eigen::Vector3f::UnitY()));
    origin_A.rotate (Eigen::AngleAxisf (origin(5), Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f pose_A = Eigen::Affine3f::Identity();
    pose_A.translation() << pose(0), pose(1), pose(2);
    pose_A.rotate (Eigen::AngleAxisf (origin(3), Eigen::Vector3f::UnitX()));
    pose_A.rotate (Eigen::AngleAxisf (origin(4), Eigen::Vector3f::UnitY()));
    pose_A.rotate (Eigen::AngleAxisf (origin(5), Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f result_A;
    result_A.matrix() = origin_A.matrix()*pose_A.matrix();
    Vector3f rot = result_A.rotation().eulerAngles(0, 1, 2);

    Vector6f result;
    result(0) = result_A.translation()(0); result(1) = result_A.translation()(1); result(2) = result_A.translation()(2);
    result(3) = rot(0); result(4) = rot(1); result(5) = rot(3);
    return result;
}

bool Kalman_update(const Vector6f &measure_X, const Matrix6f &measure_cov, 
                    Vector6f &est_X, Matrix6f &est_cov)
{
    Matrix6f Rk, Sk, Kk, Ik, Sk_inv;
    Ik.setIdentity();
    Vector6f yk;   

    Vector6f measured, estimated, residual, corrected, correction;

    measured(0) = measure_X(0); measured(1) = measure_X(1); measured(2) = measure_X(2);
    measured(3) = measure_X(3); measured(4) = measure_X(4); measured(5) = measure_X(5);

    estimated(0) = est_X(0); estimated(1) = est_X(1); estimated(2) = est_X(2);
    estimated(3) = est_X(3); estimated(4) = est_X(4); estimated(5) = est_X(5);
 
    bool invertible;
    double det;

    residual = subPose(estimated, measured); 

    Rk = measure_cov;
    Sk = est_cov + Rk;
    FullPivLU<Matrix6f> lu_decomp(Sk);
    invertible = lu_decomp.isInvertible();

    if (!invertible) 
    {
      std::cerr << "Matrix not invertible \n";
    }
    else
    {
      Sk_inv = Sk.inverse();
    }
    yk(0) = residual(0); yk(1) = residual(1); yk(2) = residual(2);
    yk(3) = residual(3); yk(4) = residual(4); yk(5) = residual(5);

    Kk = est_cov*Sk_inv;
    yk = Kk*yk;
    correction(0) = yk(0); correction(1) = yk(1); correction(2) = yk(2);
    correction(3) = yk(3); correction(4) = yk(4); correction(5) = yk(5);  
    corrected = addPose(estimated, correction);
    
    //est_X(0) = corrected(0); est_X(1) = corrected(1); est_X(2) = corrected(2);
    //est_X(3) = corrected(3); est_X(4) = corrected(4); est_X(5) = corrected(5);
    est_X = corrected;
    est_cov = (Ik - Kk)*est_cov;
}

void pose_process(std::string line, Eigen::Matrix4f &T, int class_index)
{
	  Eigen::Vector3f trans;
    float rot_quaternion[4];
    vector<string> st;
    boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
    trans(0) = std::stof(st[0]); trans(1) = std::stof(st[1]); trans(2) = std::stof(st[2]); //translaton
    rot_quaternion[0] = std::stof(st[3]); rot_quaternion[1] = std::stof(st[4]); //rotation
    rot_quaternion[2] = std::stof(st[5]); rot_quaternion[3] = std::stof(st[6]); //rotation
    float mean_dist = std::stof(st[7]);

    Eigen::Quaternionf q(rot_quaternion[0], rot_quaternion[1], rot_quaternion[2], rot_quaternion[3]);
    Eigen::Matrix4f model2cam_T = Eigen::Matrix4f::Identity();
    model2cam_T.block(0, 3, 3, 1) = trans;
    model2cam_T.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();

    Eigen::Matrix4f model2global_T = cam_T * model2cam_T;

    if(use_Kalman_Filter)
    {
      trans = model2global_T.block(0, 3, 3, 1);
      Eigen::Matrix3f rot_M = model2global_T.block(0, 0, 3, 3);
      Eigen::Vector3f rot_eulAg = rot_M.eulerAngles(0, 1, 2);
      
      Vector6f meas_pose;
      meas_pose(0) = trans(0); meas_pose(1) = trans(1); meas_pose(2) = trans(2); //translation
      meas_pose(3) = rot_eulAg(0); meas_pose(4) = rot_eulAg(1); meas_pose(5) = rot_eulAg(2); //rotation

      Matrix6f meas_cov;
      meas_cov.setIdentity();
      meas_cov = mean_dist*meas_cov;

      if(first_observe[class_index])
      {
        est_cov[class_index].setIdentity();
        est_cov[class_index] = 0.01*est_cov[class_index];
        est_pose[class_index] = meas_pose;
        first_observe[class_index] = false;
      }

      Eigen::Affine3f pose = Eigen::Affine3f::Identity();
      Kalman_update(meas_pose, meas_cov, est_pose[class_index], est_cov[class_index]);
      pose.translation() << est_pose[class_index](0), est_pose[class_index](1), est_pose[class_index](2);
      pose.rotate (Eigen::AngleAxisf (est_pose[class_index](3), Eigen::Vector3f::UnitX()));
      pose.rotate (Eigen::AngleAxisf (est_pose[class_index](4), Eigen::Vector3f::UnitY()));
      pose.rotate (Eigen::AngleAxisf (est_pose[class_index](5), Eigen::Vector3f::UnitZ()));
      T = pose.matrix();
    }
    else T = model2global_T; 
}

Eigen::Matrix4f read_cam_pose(std::string line)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Vector3f trans;
    float rot_quaternion[4];
    vector<string> st;
    boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
    trans(0) = std::stof(st[1]); trans(1) = std::stof(st[2]); trans(2) = std::stof(st[3]); //translaton
    rot_quaternion[0] = std::stof(st[4]); rot_quaternion[1] = std::stof(st[5]); //rotation
    rot_quaternion[2] = std::stof(st[6]); rot_quaternion[3] = std::stof(st[7]); //rotation

    Eigen::Quaternionf q(rot_quaternion[0], rot_quaternion[1], rot_quaternion[2], rot_quaternion[3]);
    
    T.block(0, 3, 3, 1) = trans;
    T.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();

    return T;
}

int main(int argc, char **argv)
{
  for( int i = 0; i < 11; ++i )
		first_observe[i] = true;
  frame_count = 0;
  
  ros::init(argc, argv, "object_pose_visualization");
  ros::NodeHandle nh_, cloud_n, n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("my_cloud", 1);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
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

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

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
    frame_count++;

    if (posefile.is_open())                     
    {
      if (!posefile.eof())                 
      {
        if(firstLine) 
        {
          getline (posefile, line);
          firstLine = false;
        }
        //line = "0000000001";
        depth_path = data_dir + line + "_depth.png";
        rgb_path = data_dir + line + "_rgb.png";
        getline (posefile, line);
        cam_T = read_cam_pose(line);

        while(true)
        {
          getline (posefile, line);
          //std::cerr << line <<  endl;          
          if(line.length() == 10 | line=="") break;
          int cls_index = std::stoi(line) - 1;
          std::string model_path = model_dir + classes[cls_index] + ".ply";
          //std::cerr << model_path <<  endl;          
          if(cls_index > 1) model_paths.push_back(model_path);
          getline (posefile, line);
          if(cls_index > 1)
          {
            Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
            pose_process(line, T, cls_index);
            transforms.push_back(T);
          }
          
        }
      }
    }
    else 
    {
      std::cerr << "Unable to open file";
      exit(0);
    }
  
    std_msgs::String msg;
    msg.data = "hi cuong";
    chatter_pub.publish(msg);

    loadModels();
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