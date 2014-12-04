/*
 * Copyright (C) 2014 Love Park Robotics, LLC
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

#include <cstdint>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <o3d3xx.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class O3D3xxNode
{
public:
  O3D3xxNode()
    : timeout_millis_(500),
      spinner_(new ros::AsyncSpinner(1))
  {
    std::string camera_ip;
    int xmlrpc_port;
    std::string password;

    ros::NodeHandle nh("~");
    nh.param("ip", camera_ip, o3d3xx::DEFAULT_IP);
    nh.param("xmlrpc_port", xmlrpc_port, (int) o3d3xx::DEFAULT_XMLRPC_PORT);
    nh.param("password", password, o3d3xx::DEFAULT_PASSWORD);
    nh.param("timeout_millis", this->timeout_millis_, 500);

    this->frame_id_ = ros::this_node::getName() + "_link";

    //----------------------
    // Instantiate the camera
    //----------------------
    this->cam_ =
      std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

    //----------------------
    // Published topics
    //----------------------
    this->cloud_pub_ =
      nh.advertise<pcl::PointCloud<o3d3xx::PointT> >("/cloud", 1);

    image_transport::ImageTransport it(nh);
    this->depth_pub_ = it.advertise("/depth", 1);
    this->amplitude_pub_ = it.advertise("/amplitude", 1);
    this->conf_pub_ = it.advertise("/confidence", 1);
    this->good_bad_pub_ = it.advertise("/good_bad_pixels", 1);

    //----------------------
    // Subscribed services
    //----------------------

    // set up service callbacks
    // use std::bind to do that
  }

  //----------------------
  // Publish loop
  //----------------------
  void Run()
  {
    this->spinner_->start();

    o3d3xx::FrameGrabber::Ptr fg =
      std::make_shared<o3d3xx::FrameGrabber>(this->cam_);

    o3d3xx::ImageBuffer::Ptr buff =
      std::make_shared<o3d3xx::ImageBuffer>();

    pcl::PointCloud<o3d3xx::PointT>::Ptr
      cloud(new pcl::PointCloud<o3d3xx::PointT>());

    cv::Mat confidence_img;

    while (ros::ok())
      {
	if (! fg->WaitForFrame(buff.get(), this->timeout_millis_))
	  {
	    ROS_WARN("Timeout waiting for camera!");
	    continue;
	  }

	// boost::shared_ptr vs std::shared_ptr forces us to make this copy :(
	pcl::copyPointCloud(*(buff->Cloud().get()), *cloud);
	cloud->header.frame_id = this->frame_id_;
	this->cloud_pub_.publish(cloud);

	sensor_msgs::ImagePtr depth =
	  cv_bridge::CvImage(std_msgs::Header(),
			     "mono16", buff->DepthImage()).toImageMsg();
	depth->header.frame_id = this->frame_id_;
	this->depth_pub_.publish(depth);

	sensor_msgs::ImagePtr amplitude =
	  cv_bridge::CvImage(std_msgs::Header(),
			     "mono16", buff->AmplitudeImage()).toImageMsg();
	amplitude->header.frame_id = this->frame_id_;
	this->amplitude_pub_.publish(amplitude);

	confidence_img = buff->ConfidenceImage();
	sensor_msgs::ImagePtr confidence =
	  cv_bridge::CvImage(std_msgs::Header(),
			     "mono8", confidence_img).toImageMsg();
	confidence->header.frame_id = this->frame_id_;
	this->conf_pub_.publish(confidence);

	cv::Mat good_bad_map = cv::Mat::ones(confidence_img.rows,
					     confidence_img.cols,
					     CV_8UC1);
	cv::bitwise_and(confidence_img, good_bad_map,
			good_bad_map);
	good_bad_map *= 255;
	sensor_msgs::ImagePtr good_bad =
	  cv_bridge::CvImage(std_msgs::Header(),
			     "mono8", good_bad_map).toImageMsg();
	good_bad->header.frame_id = this->frame_id_;
	this->good_bad_pub_.publish(good_bad);
      }
  }

private:
  int timeout_millis_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  o3d3xx::Camera::Ptr cam_;

  std::string frame_id_;
  ros::Publisher cloud_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher amplitude_pub_;
  image_transport::Publisher conf_pub_;
  image_transport::Publisher good_bad_pub_;

}; // end: class O3D3xxNode

int main(int argc, char **argv)
{
  o3d3xx::Logging::Init();
  ros::init(argc, argv, "o3d3xx");
  O3D3xxNode().Run();
  return 0;
}
