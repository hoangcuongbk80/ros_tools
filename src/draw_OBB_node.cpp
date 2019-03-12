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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr     OBBs (new pcl::PointCloud<pcl::PointXYZ>);
visualization_msgs::MarkerArray         multiMarker;
std::vector<string>                     ObjectNameList;
std::string                             *objectName;
std::vector<int>                        recognizedObjects;

int OBB_Estimation()
{
    if(!cloud->size())
    {
      std::cerr << "cloud is empty!" << "\n";
      return 0;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr OBB (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZRGB point;
    
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
 
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    
    float lengthOBB[3];
    lengthOBB[0] = fabs(maxPoint.x - minPoint.x); //MAX length OBB
    lengthOBB[1] = fabs(maxPoint.y - minPoint.y); //MID length OBB
    lengthOBB[2] = fabs(maxPoint.z - minPoint.z); //MIN length OBB

    pcl::PointCloud<pcl::PointXYZ>::Ptr OBB_Origin(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ OBB_points;
    OBB_points.x = -lengthOBB[0] / 2.0; OBB_points.y = -lengthOBB[1] / 2.0; OBB_points.z = -lengthOBB[2] / 2.0;
    OBB_Origin->push_back(OBB_points); // Min Point
    OBB_points.x = -lengthOBB[0] / 2.0; OBB_points.y = lengthOBB[1] / 2.0; OBB_points.z = -lengthOBB[2] / 2.0;
    OBB_Origin->push_back(OBB_points);
    OBB_points.x = -lengthOBB[0] / 2.0; OBB_points.y = lengthOBB[1] / 2.0; OBB_points.z = lengthOBB[2] / 2.0;
    OBB_Origin->push_back(OBB_points);
    OBB_points.x = -lengthOBB[0] / 2.0; OBB_points.y = -lengthOBB[1] / 2.0; OBB_points.z = lengthOBB[2] / 2.0;
    OBB_Origin->push_back(OBB_points);

    OBB_points.x = lengthOBB[0] / 2.0; OBB_points.y = lengthOBB[1] / 2.0; OBB_points.z = lengthOBB[2] / 2.0;
    OBB_Origin->push_back(OBB_points); //Max point
    OBB_points.x = lengthOBB[0] / 2.0; OBB_points.y = -lengthOBB[1] / 2.0; OBB_points.z = lengthOBB[2] / 2.0;
    OBB_Origin->push_back(OBB_points);
    OBB_points.x = lengthOBB[0] / 2.0; OBB_points.y = -lengthOBB[1] / 2.0; OBB_points.z = -lengthOBB[2] / 2.0;
    OBB_Origin->push_back(OBB_points);
    OBB_points.x = lengthOBB[0] / 2.0; OBB_points.y = lengthOBB[1] / 2.0; OBB_points.z = -lengthOBB[2] / 2.0;
    OBB_Origin->push_back(OBB_points);

    pcl::transformPointCloud(*OBB_Origin, *OBB_Origin, projectionTransform.inverse());

    if(lengthOBB[0] < lengthOBB[1])
    {
        float buf = lengthOBB[0]; lengthOBB[0] = lengthOBB[1]; lengthOBB[1] = buf;
    }
    if(lengthOBB[0] < lengthOBB[2])
    {
        float buf = lengthOBB[0]; lengthOBB[0] = lengthOBB[2]; lengthOBB[2] = buf;
    }
    if(lengthOBB[1] < lengthOBB[2])
    {
        float buf = lengthOBB[1]; lengthOBB[1] = lengthOBB[2]; lengthOBB[2] = buf;
    }
    
    *OBBs += *OBB_Origin;
    OBB_points.x = lengthOBB[0]; OBB_points.y = lengthOBB[1]; OBB_points.z = lengthOBB[2];
    OBBs->push_back(OBB_points);
    std::cerr << "OBB length: " << lengthOBB[0] << " " << lengthOBB[1] << " " << lengthOBB[2];
    return 1;
}

void draw_OBBs()
{
    visualization_msgs::Marker OBB;
    geometry_msgs::Point p;

    OBB.header.frame_id = "camera_depth_optical_frame";
    OBB.header.stamp = ros::Time::now();
    OBB.ns = "OBBs";
    OBB.id = 0;
    OBB.type = visualization_msgs::Marker::LINE_LIST;
    OBB.action = visualization_msgs::Marker::ADD;
    OBB.pose.position.x = 0;
    OBB.pose.position.y = 0;
    OBB.pose.position.z = 0;
    OBB.pose.orientation.x = 0.0;
    OBB.pose.orientation.y = 0.0;
    OBB.pose.orientation.z = 0.0;
    OBB.pose.orientation.w = 1.0;
    OBB.scale.x = 0.01; OBB.scale.y = 0.01; OBB.scale.z = 0.01;
    OBB.color.r = 1.0f; OBB.color.g = 0.0f; OBB.color.b = 0.0f; OBB.color.a = 1.0;

    visualization_msgs::Marker ObjectName;
    ObjectName.header.frame_id = "camera_depth_optical_frame";
    ObjectName.header.stamp = ros::Time::now();
    ObjectName.id = 0;
    ObjectName.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ObjectName.action = visualization_msgs::Marker::ADD;
    ObjectName.pose.orientation.x = 0.0;
    ObjectName.pose.orientation.y = 0.0;
    ObjectName.pose.orientation.z = 0.0;
    ObjectName.pose.orientation.w = 1.0;
    ObjectName.scale.x = 0.15; ObjectName.scale.y = 0.15; ObjectName.scale.z = 0.15;
    ObjectName.color.r = 0.0f; ObjectName.color.g = 0.0f; ObjectName.color.b = 1.0f; ObjectName.color.a = 1.0;
    std::cerr << "\n" << "cuong" << "\n";

    if(OBBs->size() == 0) 
    { 
      std::cerr << "No OBB data!";
      OBB.lifetime = ros::Duration();
      multiMarker.markers.push_back(OBB);
      if(!ObjectNameList.empty())
      {
        for(int i=0; i < ObjectNameList.size(); i++)
        {
          ObjectName.ns  = ObjectNameList[i];
          ObjectName.text = "Cuong";
          ObjectName.lifetime = ros::Duration();
          multiMarker.markers.push_back(ObjectName);
        }
        ObjectNameList.clear();
      }      
      return;
    };

    if(!ObjectNameList.empty())
      {
        for(int i=0; i < ObjectNameList.size(); i++)
        {
          ObjectName.ns  = ObjectNameList[i];
          ObjectName.text = "Cuong";
          ObjectName.lifetime = ros::Duration();
          multiMarker.markers.push_back(ObjectName);
        }
      }
    ObjectNameList.clear();
 
    for(int k = 0; k < recognizedObjects.size(); k++)
    {
       int begin = k*9; int stop = begin + 8;
       for(int i = begin; i < stop; i++)
       {
          if(i == begin + 3 || i == begin + 7)
          {
             p.x = OBBs->points[i].x;
             p.y = OBBs->points[i].y; p.z = OBBs->points[i].z;
             OBB.points.push_back(p);
             p.x = OBBs->points[i-3].x;
             p.y = OBBs->points[i-3].y; p.z = OBBs->points[i-3].z;
             OBB.points.push_back(p);
             if(i == begin + 3)
             {
                p.x = OBBs->points[i].x;
                p.y = OBBs->points[i].y; p.z = OBBs->points[i].z;
                OBB.points.push_back(p);
                p.x = OBBs->points[begin + 5].x;
                p.y = OBBs->points[begin + 5].y; 
                p.z = OBBs->points[begin + 5].z;
                OBB.points.push_back(p);
             }
             if(i == begin + 7)
             {
                p.x = OBBs->points[i].x;
                p.y = OBBs->points[i].y; p.z = OBBs->points[i].z;
                OBB.points.push_back(p);
                p.x = OBBs->points[begin + 1].x;
                p.y = OBBs->points[begin + 1].y; 
                p.z = OBBs->points[begin + 1].z;
                OBB.points.push_back(p);
             }
          }
          else
          {
             p.x = OBBs->points[i].x;
             p.y = OBBs->points[i].y; p.z = OBBs->points[i].z;
             OBB.points.push_back(p);
             p.x = OBBs->points[i+1].x;
             p.y = OBBs->points[i+1].y; p.z = OBBs->points[i+1].z;
             OBB.points.push_back(p);
             if(i == begin + 0)
             {
                p.x = OBBs->points[i].x;
                p.y = OBBs->points[i].y; p.z = OBBs->points[i].z;
                OBB.points.push_back(p);
                p.x = OBBs->points[begin + 6].x;
                p.y = OBBs->points[begin + 6].y; 
                p.z = OBBs->points[begin + 6].z;
                OBB.points.push_back(p);
             }
             if(i == begin + 2)
             {
                p.x = OBBs->points[i].x;
                p.y = OBBs->points[i].y; p.z = OBBs->points[i].z;
                OBB.points.push_back(p);
                p.x = OBBs->points[begin + 4].x;
                p.y = OBBs->points[begin + 4].y; 
                p.z = OBBs->points[begin + 4].z;
                OBB.points.push_back(p);
             }
          }
       }
       
       ostringstream convert;
       convert << k;
       ObjectName.ns = objectName[recognizedObjects[k]] + convert.str();
       ObjectName.pose.position.x = OBBs->points[begin + k].x;
       ObjectName.pose.position.y = OBBs->points[begin + k].y;
       ObjectName.pose.position.z = OBBs->points[begin + k].z;
       ObjectName.color.a = 1.0;
       ObjectName.text = objectName[recognizedObjects[k]];
       ObjectNameList.push_back(ObjectName.text);

       std::cerr << ObjectName.text << "\n";
       ObjectName.lifetime = ros::Duration();
       multiMarker.markers.push_back(ObjectName);
    }

    OBB.lifetime = ros::Duration();
    multiMarker.markers.push_back(OBB);

}

void colorMap(int i, pcl::PointXYZRGB &point)
{
  if (i == 1) // red  
  {
    point.r = 255;
  }
  else if (i == 2) //line
  {
    point.g = 255;
  }
  else if ( i == 3) //blue
  { 
    point.b = 255;
  } 
  else if ( i == 4) //maroon
  {
    point.r = 128;

  }
  else if ( i == 5) //green
  {
    point.g = 128;
  }  
  else if ( i == 6) //navy
  {
      point.b = 128;
  }
  else if ( i == 7) //yellow
  {
    point.r = 255;
    point.g = 255;
  }
  else if ( i == 8) //magenta
  {
    point.r = 255;
    point.b = 255;
  }
  else if ( i == 9) //cyan
  {
    point.g = 255;
    point.b = 255;
  }    
  else if ( i == 10) //olive
  {
    point.r = 128;
    point.g = 128;
  }
  else if ( i == 11) //purple
  {
    point.r = 128;
    point.b = 128;
  } 
    
  else if ( i == 12) //teal
  {
    point.g = 128;
    point.b = 128;
  }
    
  else if ( i == 13) 
  {
    point.r = 92;
    point.g = 112;
    point.b = 92;
  }
  else if ( i == 14) //brown
  {
      point.r = 165;
      point.g = 42;
      point.b = 42;
  }    
  else //silver
  {
    point.r = 192;
    point.g = 192;
    point.b = 192;
  }                   
      
}

void loadPointCloud(int i)
{
  recognizedObjects.push_back(i);
  std::string filename; 
  filename = "/home/hoang/Datasets/office-iros/" + std::to_string(i+1) + ".ply";
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (filename, *cloud);
  
   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << 1, 0.0, 1.0;
  transform_2.rotate (Eigen::AngleAxisf (-2, Eigen::Vector3f::UnitX()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*cloud, *cloud, transform_2);
  
  OBB_Estimation();
  for(int k=0; k<cloud->size(); k++)
  {
    pcl::PointXYZRGB point;
    point.x = cloud->points[k].x;
    point.y = cloud->points[k].y;
    point.z = cloud->points[k].z;
    point.r=0; point.b=0; point.g=0;
    colorMap(i+1, point);
    myCloud->push_back(point);
  }
  std::cerr << "myCloud: " << myCloud->size() << "\n";
}

void loadBackGround()
{
  std::string filename; 
  filename = "/home/hoang/Datasets/office-iros/background.ply";
  pcl::io::loadPLYFile<pcl::PointXYZRGB> (filename, *cloud);
  
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << 1, 0.0, 1.0;
  transform_2.rotate (Eigen::AngleAxisf (-2, Eigen::Vector3f::UnitX()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*cloud, *cloud, transform_2);
  
  for(int k=0; k<cloud->size(); k++)
  {
    pcl::PointXYZRGB point;
    point.x = cloud->points[k].x;
    point.y = cloud->points[k].y;
    point.z = cloud->points[k].z;
    point.r=192; point.b=192; point.g=192;
    myCloud->push_back(cloud->points[k]);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OBB_Drawer");
  ros::NodeHandle cloud_n;
  ros::NodeHandle obb_n;
  ros::Publisher cloud_pub = cloud_n.advertise<sensor_msgs::PointCloud2> ("myCloud", 1);
  ros::Publisher OBBs_pub = obb_n.advertise<visualization_msgs::MarkerArray>( "OBBs", 1);
  ros::Rate loop_rate(10);

  int numOfObjects = 11; 
  objectName = new std::string[numOfObjects];
  objectName[0] = "Chair_01";  objectName[1] = "Chair_02"; 
  objectName[2] = "Chair_03";  objectName[3] = "Chair_04"; 
  objectName[4] = "Laptop_01";  objectName[5] = "Monitor_01"; 
  objectName[6] = "Monitor_02";  objectName[7] = "Monitor_03";
  objectName[8] = "Keyboard_01";  objectName[9] = "Keyboard_02"; 
  objectName[10] = "Keyboard_03";
  loadBackGround();
  for(int i=0; i<10; i++)
  {
    loadPointCloud(i);
  }
  draw_OBBs();

  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;
  myCloud->header.frame_id = "camera_depth_optical_frame";
  pcl::toPCLPointCloud2(*myCloud, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  
  while (ros::ok())
  {  
    cloud_pub.publish (output);
    OBBs_pub.publish(multiMarker);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}