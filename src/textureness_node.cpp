#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

cv::Mat rgb_img, depth_img;

void process_depth(std::string saved_depth_dir)
{
    int w = 2;
    cv::Mat weight_img(cv::Size(depth_img.cols-w, depth_img.rows-w), CV_32FC1);
    weight_img = 0;
    for(int row=w; row < depth_img.rows-w; row++)
    {
       for(int col=w; col < depth_img.cols-w; col++)       
        {
          if(isnan(depth_img.at<ushort>(row, col))) {}
          cv::Rect my_patch( col-2, row-2, w, w );
          cv::Mat img_roi = depth_img(my_patch);
          Scalar mean, stdDev;
          meanStdDev(img_roi, mean, stdDev);
          weight_img.at<float>(row, col) = stdDev[0] / (stdDev[0] + 0.002);
          //std::cerr << mean << "\n";
        }
    }
    cv::normalize(weight_img, weight_img, 0, 155, cv::NORM_MINMAX);
    weight_img = weight_img + 100;
    cv::imwrite(saved_depth_dir, weight_img);
}

void process_color(std::string saved_rgb_dir)
{
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);

    int w = 2;
    cv::Mat weight_img(cv::Size(gray_img.cols-w, gray_img.rows-w), CV_32FC1);
    weight_img = 0;
    for(int row=w; row < gray_img.rows-w; row++)
    {
       for(int col=w; col < gray_img.cols-w; col++)       
        {
          cv::Rect my_patch( col-2, row-2, w, w );
          cv::Mat img_roi = gray_img(my_patch);
          Scalar mean, stdDev;
          meanStdDev(img_roi, mean, stdDev);
          weight_img.at<float>(row, col) = stdDev[0] / (stdDev[0] + 1);
          //std::cerr << stdDev[0] << "\n";
        }
    }
    cv::normalize(weight_img, weight_img, 0, 155, cv::NORM_MINMAX);
    weight_img = weight_img + 100;   
    cv::imwrite(saved_rgb_dir, weight_img);
}

void read_images(std::string rgb_dir,std::string depth_dir)
{
    depth_img = cv::imread(depth_dir, -1);
    std::cerr << "depth type: " << depth_img.type() << "\n";
    rgb_img = cv::imread(rgb_dir, -1);
    cv::imshow("RGB image", rgb_img);
    cv::waitKey(3);
    cv::imshow("depth image", depth_img);
    cv::waitKey(3);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimate textureness");

  ros::NodeHandle nh_;

  std::string rgb_dir, depth_dir;
  std::string saved_rgb_dir, saved_depth_dir;
  nh_ = ros::NodeHandle("~");
  nh_.getParam("rgb_dir", rgb_dir);
  nh_.getParam("depth_dir", depth_dir);
  nh_.getParam("saved_rgb_dir", saved_rgb_dir);
  nh_.getParam("saved_depth_dir", saved_depth_dir);

  read_images(rgb_dir, depth_dir);
  process_color(saved_rgb_dir);
  process_depth(saved_depth_dir);
  return 0;
}
