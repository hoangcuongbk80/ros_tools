#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

using namespace cv;
using namespace std;

class readrgbdNode
{
  public:
    readrgbdNode();
    virtual ~readrgbdNode();
    void subcribeTopics();
    void advertiseTopics();
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void cloudPublish();

    bool image_save;
    std::string depth_topsub, rgb_topsub, cloud_topsub, cloud_toppub;
    std::string saved_rgb_dir, saved_depth_dir;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;


  private:
   ros::NodeHandle nh_, nh_rgb, nh_depth, nh_cloud;
   ros::Subscriber depth_sub, rgb_sub, cloud_sub;
   ros::Publisher cloud_pub;
};

readrgbdNode::readrgbdNode()
{
  nh_ = ros::NodeHandle("~");
  nh_rgb = ros::NodeHandle("~");
  nh_depth = ros::NodeHandle("~");
  nh_cloud = ros::NodeHandle("~");

  nh_depth.getParam("depth_topsub", depth_topsub);
  nh_rgb.getParam("rgb_topsub", rgb_topsub);
  nh_cloud.getParam("cloud_topsub", cloud_topsub);
  nh_.getParam("cloud_toppub", cloud_toppub);

  nh_.getParam("image_save", image_save);
  nh_.getParam("saved_rgb_dir", saved_rgb_dir);
  nh_.getParam("saved_depth_dir", saved_depth_dir);

  std::cerr << "depth topics sub: " << "\n" << depth_topsub << "\n";
  std::cerr << "rgb topics sub: " << "\n" << rgb_topsub << "\n";
  std::cerr << "cloud topics sub: " << "\n" << cloud_topsub << "\n";
  std::cerr << "save depth to: " << "\n" << saved_depth_dir << "\n";
  std::cerr << "save rgb to: " << "\n" << saved_rgb_dir << "\n";

  cloud_ .reset(new pcl::PointCloud<pcl::PointXYZ>);

}

readrgbdNode::~readrgbdNode()
{
};

void readrgbdNode::subcribeTopics()
{
  depth_sub = nh_depth.subscribe (depth_topsub, 1, &readrgbdNode::depthCallback, this);
  rgb_sub = nh_rgb.subscribe (rgb_topsub, 1, &readrgbdNode::rgbCallback, this);  
  cloud_sub = nh_cloud.subscribe (cloud_topsub, 1, &readrgbdNode::cloudCallback, this);
}

void readrgbdNode::advertiseTopics()
{
  cloud_pub = nh_.advertise<sensor_msgs::PointCloud2> (cloud_toppub, 1);
}

void readrgbdNode::depthCallback (const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr bridge;

  try
  {
    bridge = cv_bridge::toCvCopy(msg, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform depth image.");
    return;
  }

  cv::Mat depth_img;
  depth_img = bridge->image.clone();
  depth_img.convertTo(depth_img, CV_16UC1, 1000.0);
  cv::imshow("depth", depth_img);
  if(image_save) cv::imwrite( saved_depth_dir, depth_img );
  cv::waitKey(3);
}

void readrgbdNode::rgbCallback (const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(msg, "bgr8");    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform rgb image.");
    return;
  }
  cv::Mat rgb_image;
  rgb_image = bridge->image;
  cv::imshow("RGB image", rgb_image);
  if(image_save) cv::imwrite( saved_rgb_dir, rgb_image );
  cv::waitKey(3);
}

void readrgbdNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{   
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
  pcl_conversions::toPCL(*cloud_msg, *cloud2);
  
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->header.frame_id = "camera_depth_optical_frame";
  pcl::fromPCLPointCloud2( *cloud2, *cloud_);
  // do something on cloud_ then  
  pcl::toPCLPointCloud2(*cloud_, *cloud2);

  sensor_msgs::PointCloud2 output;  
  pcl_conversions::fromPCL(*cloud2, output);
  cloud_pub.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_transform");
  readrgbdNode mainNode;
  mainNode.subcribeTopics();
  mainNode.advertiseTopics();
  ros::spin();
  return 0;
}
