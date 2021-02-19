#include <iostream> 
#include <cstdlib> 
#include <stdio.h>
#include <stdlib.h>

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper (new pcl::PointCloud<pcl::PointXYZ>);
visualization_msgs::MarkerArray         multiMarker;
 vector<vector<float> >                 grasps;


void gripper_init()
{
/*       
       p4 -         - p5             Z -     - Y
          -         -                  -   -
          -   p1    -                  - -
          - - - - - -                  - - - - - X
          p2   -    p3
               -
               -p0

 */  
  pcl::PointXYZ p0;  
  p0.x = 0; p0.y = 0; p0.z = -0.1;
  gripper->push_back(p0);
  
  pcl::PointXYZ p1;  
  p1.x = 0; p1.y = 0; p1.z = 0;
  gripper->push_back(p1);

  pcl::PointXYZ p2;  
  p2.x = -0.05; p2.y = 0; p2.z = 0;
  gripper->push_back(p2);

  pcl::PointXYZ p3;  
  p3.x = 0.05; p3.y = 0; p3.z = 0;
  gripper->push_back(p3);

  pcl::PointXYZ p4;  
  p4.x = -0.05; p4.y = 0; p4.z = 0.1;
  gripper->push_back(p4);

  pcl::PointXYZ p5;  
  p5.x = 0.05; p5.y = 0; p5.z = 0.1;
  gripper->push_back(p5);
}

bool setup_marker()
{
    if(grasps.empty())
    {
      std::cerr << "\n Grasps empty!";
      return false;
    }

    for(int i=0; i < grasps.size(); i++)
    {
      visualization_msgs::Marker line_list;
      geometry_msgs::Point p;

      line_list.header.frame_id = "camera_depth_optical_frame";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "grasps";
      line_list.id = i;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.position.x = grasps[i][0];
      line_list.pose.position.y = grasps[i][1];
      line_list.pose.position.z = grasps[i][2];
      line_list.pose.orientation.x = grasps[i][3];
      line_list.pose.orientation.y = grasps[i][4];
      line_list.pose.orientation.z = grasps[i][5];
      line_list.pose.orientation.w = grasps[i][6];
      line_list.scale.x = 0.002; line_list.scale.y = 0.002; line_list.scale.z = 0.002;
      line_list.color.r = 1.0f; line_list.color.g = 0.0f; line_list.color.b = 0.0f; line_list.color.a = 1.0;
      
      p.x = gripper->points[0].x; p.y = gripper->points[0].y; p.z = gripper->points[0].z;
      line_list.points.push_back(p);

      p.x = gripper->points[1].x; p.y = gripper->points[1].y; p.z = gripper->points[1].z;
      line_list.points.push_back(p);

      p.x = gripper->points[2].x; p.y = gripper->points[2].y; p.z = gripper->points[2].z;
      line_list.points.push_back(p);

      p.x = gripper->points[3].x; p.y = gripper->points[3].y; p.z = gripper->points[3].z;
      line_list.points.push_back(p);

      p.x = gripper->points[2].x; p.y = gripper->points[2].y; p.z = gripper->points[2].z;
      line_list.points.push_back(p);

      p.x = gripper->points[4].x; p.y = gripper->points[4].y; p.z = gripper->points[4].z;
      line_list.points.push_back(p);

      p.x = gripper->points[3].x; p.y = gripper->points[3].y; p.z = gripper->points[3].z;
      line_list.points.push_back(p);

      p.x = gripper->points[5].x; p.y = gripper->points[5].y; p.z = gripper->points[5].z;
      line_list.points.push_back(p);

      line_list.lifetime = ros::Duration();
      multiMarker.markers.push_back(line_list);
    }
    return true;
}

void read_grasp(std::string grasp_path)
{
  ifstream grasp_file (grasp_path);
  if (grasp_file.is_open())            
    {
      string line;
      getline (grasp_file, line);
       while(!grasp_file.eof())
      {
        vector<string> st;
        vector<float> grasp;
        getline (grasp_file, line);
        boost::trim(line);
		    boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
        if(st.size() < 6) continue;
        for(int i=0; i < st.size(); i++)
          {
            grasp.push_back(std::stof(st[i]));
          }
        grasps.push_back(grasp);
        
        //trans(0) = std::stof(st[4]); trans(1) = std::stof(st[5]); trans(2) = std::stof(st[6]); //translaton
        //rot_quaternion[0] = std::stof(st[0]); rot_quaternion[1] = std::stof(st[1]); //rotation
        //rot_quaternion[2] = std::stof(st[2]); rot_quaternion[3] = std::stof(st[3]); //rotation
      }
    }
  else 
    {
      std::cerr << "Unable to open file";
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Grasp_Visualization");
  std::cerr << "\n"<< "---------------------Grasp Visualization---------------------" << "\n";
  ros::NodeHandle nh_, cloud_n, grasp_n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("myCloud", 1);
  ros::Publisher grasp_pub = grasp_n.advertise<visualization_msgs::MarkerArray>( "myGrasp", 1);
  ros::Rate loop_rate(10);

  std:string pc_path, grasp_path;
  
  nh_ = ros::NodeHandle("~");
  nh_.getParam("point_cloud_path", pc_path);
  nh_.getParam("grasp_path", grasp_path);
  
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (pc_path, *myCloud);
  gripper_init();
  read_grasp(grasp_path);
  setup_marker();

  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;
  myCloud->header.frame_id = "camera_depth_optical_frame";
  pcl::toPCLPointCloud2(*myCloud, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  
 
  while (ros::ok())
  {  
    cloud_pub.publish (output);
    grasp_pub.publish(multiMarker);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}