/*
 * Copyright (C) 2015 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <o3d3xx/image.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "o3d3xx/Dump.h"

class O3D3xxFileWriterNode
{
public:
  O3D3xxFileWriterNode()
    : spinner_(new ros::AsyncSpinner(4)),
      outdir_("/tmp"),
      dump_yaml_(false),
      cloud_idx_(0),
      depth_idx_(0),
      amplitude_idx_(0),
      raw_amplitude_idx_(0),
      confidence_idx_(0),
      xyz_idx_(0)
  {
    ros::NodeHandle nh("~");
    nh.param("outdir", this->outdir_, std::string("/tmp"));
    nh.param("dump_yaml", this->dump_yaml_, false);

    // make sure the output directories exist
    std::vector<std::string> dirs =
      {"cloud", "depth", "amplitude", "confidence", "xyz_image",
       "raw_amplitude"};

    for (auto& dir : dirs)
    {
      std::string target_dir = this->outdir_ + "/" + dir;

      if (!boost::filesystem::create_directories(target_dir))
      {
        if (boost::filesystem::is_directory(target_dir))
        {
          throw std::runtime_error("Directory already exists: " +
                                   target_dir);
        }
        else
        {
          throw std::runtime_error("Could not create directory: " +
                                   target_dir);
        }
      }
    }

    //----------------------
    // Subscribed topics
    //----------------------
    this->cloud_sub_ =
      nh.subscribe<pcl::PointCloud<o3d3xx::PointT> >
      ("/cloud", 10,
       std::bind(&O3D3xxFileWriterNode::CloudCb, this,
                 std::placeholders::_1));

    this->depth_sub_ =
      nh.subscribe<sensor_msgs::Image>
      ("/depth", 10,
       std::bind(&O3D3xxFileWriterNode::ImageCb, this,
                 std::placeholders::_1, "depth"));

    this->amplitude_sub_ =
      nh.subscribe<sensor_msgs::Image>
      ("/amplitude", 10,
       std::bind(&O3D3xxFileWriterNode::ImageCb, this,
                 std::placeholders::_1, "amplitude"));

    this->raw_amplitude_sub_ =
      nh.subscribe<sensor_msgs::Image>
      ("/raw_amplitude", 10,
       std::bind(&O3D3xxFileWriterNode::ImageCb, this,
                 std::placeholders::_1, "raw_amplitude"));

    this->confidence_sub_ =
      nh.subscribe<sensor_msgs::Image>
      ("/confidence", 10,
       std::bind(&O3D3xxFileWriterNode::ImageCb, this,
                 std::placeholders::_1, "confidence"));

    this->xyz_sub_ =
      nh.subscribe<sensor_msgs::Image>
      ("/xyz_image", 10,
       std::bind(&O3D3xxFileWriterNode::ImageCb, this,
                 std::placeholders::_1, "xyz_image"));

    //----------------------
    // Call the camera's `Dump` service, and (try to) write the
    // current camera configuration to the output directory
    //----------------------
    ROS_INFO("Calling `Dump' service...");
    ros::ServiceClient client = nh.serviceClient<o3d3xx::Dump>("/Dump");

    o3d3xx::Dump srv;
    std::string json_file = this->outdir_ + "/o3d3xx.json";
    std::ofstream out(json_file);
    if (client.call(srv))
      {
        out << srv.response.config;
        ROS_INFO("Wrote camera configuration to: %s", json_file.c_str());
      }
    else
      {
        ROS_ERROR("Failed to call `Dump` service!");
        ROS_WARN("Not writing camera configuration to output directory");
      }
  }

  void Run()
  {
    ROS_INFO("Starting data capture...");
    this->spinner_->start();
    ros::waitForShutdown();
  }

  /**
   * Callback on the "/cloud" topic
   */
  void CloudCb(const pcl::PointCloud<o3d3xx::PointT>::ConstPtr& cloud)
  {
    this->cloud_idx_mutex_.lock();
    int this_idx = this->cloud_idx_;
    this->cloud_idx_++;
    this->cloud_idx_mutex_.unlock();

    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << this_idx;
    pcl::io::savePCDFileASCII(this->outdir_ + "/cloud/cloud_"
                              + ss.str() + ".pcd",
                              *cloud);
  }

  /**
   * Callback on the "/xyz_image",
   * "/depth", "/amplitude", "/raw_amplitdue", and "/confidence" topics
   */
  void ImageCb(const sensor_msgs::Image::ConstPtr& im,
               const std::string& im_type)
  {
    std::string target_file = this->outdir_;
    int this_idx = 0;
    cv_bridge::CvImagePtr cv_ptr;

    if (im_type == "depth")
      {
        this->depth_idx_mutex_.lock();
        this_idx = this->depth_idx_;
        this->depth_idx_++;
        this->depth_idx_mutex_.unlock();

        target_file += "/depth/depth_";
        cv_ptr = cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::MONO16);
      }
    else if (im_type == "amplitude")
      {
        this->amplitude_idx_mutex_.lock();
        this_idx = this->amplitude_idx_;
        this->amplitude_idx_++;
        this->amplitude_idx_mutex_.unlock();

        target_file += "/amplitude/amplitude_";
        cv_ptr = cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::MONO16);
      }
    else if (im_type == "raw_amplitude")
      {
        this->raw_amplitude_idx_mutex_.lock();
        this_idx = this->raw_amplitude_idx_;
        this->raw_amplitude_idx_++;
        this->raw_amplitude_idx_mutex_.unlock();

        target_file += "/raw_amplitude/raw_amplitude_";
        cv_ptr = cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::MONO16);
      }
    else if (im_type == "confidence")
      {
        this->confidence_idx_mutex_.lock();
        this_idx = this->confidence_idx_;
        this->confidence_idx_++;
        this->confidence_idx_mutex_.unlock();

        target_file += "/confidence/confidence_";
        cv_ptr = cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::MONO8);
      }
    else if (im_type == "xyz_image")
      {
        this->xyz_idx_mutex_.lock();
        this_idx = this->xyz_idx_;
        this->xyz_idx_++;
        this->xyz_idx_mutex_.unlock();

        target_file += "/xyz_image/xyz_";
        cv_ptr =
          cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::TYPE_16SC3);
      }
    else
      {
        return;
      }

    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << this_idx;
    target_file += ss.str();

    if (this->dump_yaml_ || im_type == "xyz_image")
    {
      cv::FileStorage storage(target_file + ".yml",
                              cv::FileStorage::WRITE);
      storage << "img" << cv_ptr->image;
      storage.release();
    }

    if (im_type != "xyz_image")
      {
        imwrite(target_file + ".png", cv_ptr->image);
      }
  }

private:
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::string outdir_;
  bool dump_yaml_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber amplitude_sub_;
  ros::Subscriber raw_amplitude_sub_;
  ros::Subscriber confidence_sub_;
  ros::Subscriber xyz_sub_;

  int cloud_idx_;
  int depth_idx_;
  int amplitude_idx_;
  int raw_amplitude_idx_;
  int confidence_idx_;
  int xyz_idx_;

  std::mutex cloud_idx_mutex_;
  std::mutex depth_idx_mutex_;
  std::mutex amplitude_idx_mutex_;
  std::mutex raw_amplitude_idx_mutex_;
  std::mutex confidence_idx_mutex_;
  std::mutex xyz_idx_mutex_;

}; // end: class O3D3xxFileWriterNode

int main(int argc, char **argv)
{
  ros::init(argc, argv, "o3d3xx_file_writer");
  O3D3xxFileWriterNode().Run();
  return 0;
}
