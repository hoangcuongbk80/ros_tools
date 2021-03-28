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

#include <cmath>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  votes (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<float> pose;

void colorMap(int i, pcl::PointXYZRGB &point)
{
  if (i == 1) // red  
  {
    point.r = 255; point.g = 0; point.b = 0; 
    point.r = 192; point.g = 192; point.b = 192;
  }
  else if (i == 2) //line
  {
    point.r = 0; point.g = 255; point.b = 0;
  }
  else if ( i == 3) //blue
  { 
    point.r = 0; point.g = 0; point.b = 255;
    point.r = 92; point.g = 112; point.b = 92;
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

void colorPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int i)
{
  for(int k=0; k < cloud.size(); k++)
  {
    int j;
    if(cloud.points[k].r==255) j=0;
    if(cloud.points[k].g==255) j=1;
    if(cloud.points[k].b==255) j=2;
    int ind = i*3+j+1;
    colorMap(ind, cloud.points[k]);
    //colorMap(i+4, cloud.points[k]);
  }
}

void generate_object_votes()
{ 
    Eigen::Vector4f Cen;
    pcl::compute3DCentroid(*cloud, Cen);
    //votes.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    int rand_i;
    float rand_x, rand_y, rand_z;

    for (int i=0; i < 200; i++)
    {
      pcl::PointXYZRGB point;
      float max_r = 0.03; 
      rand_i = (-1)^i * rand();
      rand_x = static_cast <float> (rand_i) / static_cast <float> (RAND_MAX / max_r);
      rand_i = (-1)^i * rand();
      rand_y = static_cast <float> (rand_i) / static_cast <float> (RAND_MAX / max_r);
      rand_i = (-1)^i * rand();
      rand_z = static_cast <float> (rand_i) / static_cast <float> (RAND_MAX / max_r);
      
      point.x = Cen[0]+rand_x; point.y = Cen[1]+rand_y; point.z = Cen[2]+rand_z;
      votes->push_back(point);
    }

}

void generate_part_votes()
{
  //votes.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int j=0; j < 3; j++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  part (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int k=0; k < cloud->size(); k++)
    {
      if(j==0)
      {
        if(cloud->points[k].r>200)
          part->push_back(cloud->points[k]);
      }
      if(j==1)
      {
        if(cloud->points[k].g>200)
          part->push_back(cloud->points[k]);
      }
      if(j==2)
      {
        if(cloud->points[k].b>200)
          part->push_back(cloud->points[k]);
      }
    }
    Eigen::Vector4f Cen;
    pcl::compute3DCentroid(*part, Cen);
    int rand_i;
    float rand_x, rand_y, rand_z;

    for (int i=0; i < 100; i++)
    {
      pcl::PointXYZRGB point;
      float max_r = 0.01;
      rand_i = (-1)^i * rand();
      rand_x = static_cast <float> (rand_i) / static_cast <float> (RAND_MAX / max_r);
      rand_i = (-1)^i * rand();
      rand_y = static_cast <float> (rand_i) / static_cast <float> (RAND_MAX / max_r);
      rand_i = (-1)^i * rand();
      rand_z = static_cast <float> (rand_i) / static_cast <float> (RAND_MAX / max_r);
      
      point.x = Cen[0]+rand_x; point.y = Cen[1]+rand_y; point.z = Cen[2]+rand_z;
      votes->push_back(point);
    }
  }

}

void transform_points()
{
   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  transform_2.translation() << pose[0], pose[1], pose[2];
  transform_2.rotate (Eigen::AngleAxisf (pose[3], Eigen::Vector3f::UnitX()));
  transform_2.rotate (Eigen::AngleAxisf (pose[4], Eigen::Vector3f::UnitY()));
  transform_2.rotate (Eigen::AngleAxisf (pose[5], Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud (*cloud, *cloud, transform_2);
}

void loadPointCloud(std::string data_dir, std::string object_name)
{
  std::string data_path = data_dir + object_name + ".ply";
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (data_path, *cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_color");
  std::cerr << "\n"<< "---------------------add color to point cloud---------------------" << "\n";
  ros::NodeHandle nh_, cloud_n, obb_n;
  ros::Publisher vote_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("votes", 1);
  ros::Publisher input_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("input", 1);
  ros::Rate loop_rate(10);

  std:string data_dir;
  std::vector<std::string> objects_name;

  bool load_background, color_segment;

  nh_ = ros::NodeHandle("~");
  nh_.getParam("data_dir", data_dir);
  nh_.getParam("objects_name", objects_name);
  nh_.getParam("pose_list", pose);

  for(int i=0; i < objects_name.size(); ++i)
  {
    std::string pose_str = "pose_" + std::to_string(i);
    nh_.getParam(pose_str, pose);
    
    loadPointCloud(data_dir, objects_name[i]);
    transform_points();

    //generate_object_votes();    
    generate_part_votes();

    colorPointCloud(*cloud, i);
    *myCloud += *cloud;


    //loadPointCloud(data_dir, "ds1");
    //transform_points();
    //*myCloud += *cloud;
  }

  std::string saved_path = data_dir + "scene.ply";
  pcl::io::savePLYFileBinary (saved_path, *myCloud);


  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output1;
  myCloud->header.frame_id = "map";
  pcl::toPCLPointCloud2(*myCloud, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output1);

  sensor_msgs::PointCloud2 output2;
  votes->header.frame_id = "map";
  pcl::toPCLPointCloud2(*votes, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output2);
  
  while (ros::ok())
  {  
    vote_pub.publish (output2);
    input_pub.publish (output1);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}