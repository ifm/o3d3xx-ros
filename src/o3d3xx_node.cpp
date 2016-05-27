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
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <o3d3xx_camera.h>
#include <o3d3xx_framegrabber.h>
#include <o3d3xx_image.h>
#include <o3d3xx/Config.h>
#include <o3d3xx/Dump.h>
#include <o3d3xx/GetVersion.h>
#include <o3d3xx/Rm.h>
#include <o3d3xx/Extrinsics.h>
#include <o3d3xx/Trigger.h>

class O3D3xxNode
{
public:
  O3D3xxNode()
    : schema_mask_(o3d3xx::DEFAULT_SCHEMA_MASK),
      timeout_millis_(500),
      timeout_tolerance_secs_(5.0),
      publish_viz_images_(false),
      assume_sw_triggered_(false),
      spinner_(new ros::AsyncSpinner(1))
  {
    std::string camera_ip;
    int xmlrpc_port;
    std::string password;
    int schema_mask;
    std::string frame_id_base;

    ros::NodeHandle nh; // public
    ros::NodeHandle np("~"); // private

    np.param("ip", camera_ip, o3d3xx::DEFAULT_IP);
    np.param("xmlrpc_port", xmlrpc_port, (int) o3d3xx::DEFAULT_XMLRPC_PORT);
    np.param("password", password, o3d3xx::DEFAULT_PASSWORD);
    np.param("schema_mask", schema_mask, (int) o3d3xx::DEFAULT_SCHEMA_MASK);
    np.param("timeout_millis", this->timeout_millis_, 500);
    np.param("timeout_tolerance_secs", this->timeout_tolerance_secs_, 5.0);
    np.param("publish_viz_images", this->publish_viz_images_, false);
    np.param("assume_sw_triggered", this->assume_sw_triggered_, false);
    np.param("frame_id_base", frame_id_base,
             std::string(ros::this_node::getName()).substr(1));

    this->schema_mask_ = static_cast<std::uint16_t>(schema_mask);

    this->frame_id_ = frame_id_base + "_link";
    this->optical_frame_id_ = frame_id_base + "_optical_link";

    //-----------------------------------------
    // Instantiate the camera and frame-grabber
    //-----------------------------------------
    this->cam_ =
      std::make_shared<o3d3xx::Camera>(camera_ip, xmlrpc_port, password);

    // NOTE: we initially only want to stream in the unit vectors, we switch
    // to the requested mask, *after* we publish the unit vectors at least
    // once.
    this->fg_ =
      std::make_shared<o3d3xx::FrameGrabber>(this->cam_, o3d3xx::IMG_UVEC);

    //----------------------
    // Published topics
    //----------------------
    this->cloud_pub_ =
      nh.advertise<pcl::PointCloud<o3d3xx::PointT> >("cloud", 1);

    image_transport::ImageTransport it(nh);
    this->depth_pub_ = it.advertise("depth", 1);
    this->depth_viz_pub_ = it.advertise("depth_viz", 1);
    this->amplitude_pub_ = it.advertise("amplitude", 1);
    this->raw_amplitude_pub_ = it.advertise("raw_amplitude", 1);
    this->conf_pub_ = it.advertise("confidence", 1);
    this->good_bad_pub_ = it.advertise("good_bad_pixels", 1);
    this->xyz_image_pub_ = it.advertise("xyz_image", 1);

    // NOTE: not using ImageTransport here ... having issues with the
    // latching. I need to investigate further. A "normal" publisher seems to
    // work.
    this->uvec_pub_ =
      nh.advertise<sensor_msgs::Image>("unit_vectors", 1, true);

    this->extrinsics_pub_ = nh.advertise<o3d3xx::Extrinsics>("extrinsics", 1);

    //----------------------
    // Advertised services
    //----------------------
    this->version_srv_ =
      nh.advertiseService<o3d3xx::GetVersion::Request,
                          o3d3xx::GetVersion::Response>
      ("GetVersion", std::bind(&O3D3xxNode::GetVersion, this,
        std::placeholders::_1,
        std::placeholders::_2));

    this->dump_srv_ =
      nh.advertiseService<o3d3xx::Dump::Request, o3d3xx::Dump::Response>
      ("Dump", std::bind(&O3D3xxNode::Dump, this,
                          std::placeholders::_1,
                          std::placeholders::_2));

    this->config_srv_ =
      nh.advertiseService<o3d3xx::Config::Request, o3d3xx::Config::Response>
      ("Config", std::bind(&O3D3xxNode::Config, this,
                            std::placeholders::_1,
                            std::placeholders::_2));

    this->rm_srv_ =
      nh.advertiseService<o3d3xx::Rm::Request, o3d3xx::Rm::Response>
      ("Rm", std::bind(&O3D3xxNode::Rm, this,
                        std::placeholders::_1,
                        std::placeholders::_2));

    this->trigger_srv_ =
      nh.advertiseService<o3d3xx::Trigger::Request, o3d3xx::Trigger::Response>
      ("Trigger", std::bind(&O3D3xxNode::Trigger, this,
                            std::placeholders::_1,
                            std::placeholders::_2));
  }

  /**
   * Main publishing loop
   */
  void Run()
  {
    std::unique_lock<std::mutex> fg_lock(this->fg_mutex_, std::defer_lock);
    this->spinner_->start();

    o3d3xx::ImageBuffer::Ptr buff =
      std::make_shared<o3d3xx::ImageBuffer>();

    pcl::PointCloud<o3d3xx::PointT>::Ptr
      cloud(new pcl::PointCloud<o3d3xx::PointT>());

    cv::Mat confidence_img;
    cv::Mat depth_img;
    cv::Mat depth_viz_img;
    double min, max;

    std::vector<float> extrinsics(6);

    ros::Time last_frame = ros::Time::now();

    bool got_uvec = false;

    while (ros::ok())
    {
      fg_lock.lock();
      if (!this->fg_->WaitForFrame(buff.get(), this->timeout_millis_))
      {
        fg_lock.unlock();
        if (! this->assume_sw_triggered_)
          {
            ROS_WARN("Timeout waiting for camera!");
          }
        else
          {
            ros::Duration(.001).sleep();
          }

        if ((ros::Time::now() - last_frame).toSec() >
            this->timeout_tolerance_secs_)
          {
            ROS_WARN("Attempting to restart frame grabber...");
            fg_lock.lock();
            if (got_uvec)
              {
                this->fg_.reset(new o3d3xx::FrameGrabber(this->cam_,
                                                         this->schema_mask_));
              }
            else
              {
                this->fg_.reset(new o3d3xx::FrameGrabber(this->cam_,
                                                         o3d3xx::IMG_UVEC));
              }
            fg_lock.unlock();
            last_frame = ros::Time::now();
          }
        continue;
      }
      fg_lock.unlock();

      std_msgs::Header head = std_msgs::Header();
      head.stamp = ros::Time::now();
      head.frame_id = this->frame_id_;
      last_frame = head.stamp;

      std_msgs::Header optical_head = std_msgs::Header();
      optical_head.stamp = head.stamp;
      optical_head.frame_id = this->optical_frame_id_;

      // publish unit vectors once on a latched topic, then re-initialize the
      // framegrabber with the user's requested schema mask
      if (! got_uvec)
        {
          sensor_msgs::ImagePtr uvec_msg =
            cv_bridge::CvImage(optical_head,
                               sensor_msgs::image_encodings::TYPE_32FC3,
                               buff->UnitVectors()).toImageMsg();
          this->uvec_pub_.publish(uvec_msg);

          ROS_INFO("Got unit vectors, restarting framegrabber with mask: %d",
                   (int) this->schema_mask_);

          fg_lock.lock();
          got_uvec = true;
          this->fg_.reset(new o3d3xx::FrameGrabber(this->cam_,
                                                   this->schema_mask_));
          fg_lock.unlock();

          continue;
        }

      // boost::shared_ptr vs std::shared_ptr forces us to make this copy :(
      pcl::copyPointCloud(*(buff->Cloud().get()), *cloud);
      cloud->header = pcl_conversions::toPCL(head);
      this->cloud_pub_.publish(cloud);

      depth_img = buff->DepthImage();
      sensor_msgs::ImagePtr depth =
        cv_bridge::CvImage(optical_head,
                           "mono16",
                           depth_img).toImageMsg();
      this->depth_pub_.publish(depth);

      sensor_msgs::ImagePtr amplitude =
        cv_bridge::CvImage(optical_head,
                           "mono16",
                           buff->AmplitudeImage()).toImageMsg();
      this->amplitude_pub_.publish(amplitude);

      sensor_msgs::ImagePtr raw_amplitude =
        cv_bridge::CvImage(optical_head,
                           "mono16",
                           buff->RawAmplitudeImage()).toImageMsg();
      this->raw_amplitude_pub_.publish(raw_amplitude);

      confidence_img = buff->ConfidenceImage();
      sensor_msgs::ImagePtr confidence =
        cv_bridge::CvImage(optical_head,
                           "mono8",
                           confidence_img).toImageMsg();
      this->conf_pub_.publish(confidence);

      sensor_msgs::ImagePtr xyz_image_msg =
        cv_bridge::CvImage(head,
                           sensor_msgs::image_encodings::TYPE_16SC3,
                           buff->XYZImage()).toImageMsg();
      this->xyz_image_pub_.publish(xyz_image_msg);

      extrinsics = buff->Extrinsics();
      o3d3xx::Extrinsics extrinsics_msg;
      extrinsics_msg.header = optical_head;
      try
        {
          extrinsics_msg.tx = extrinsics.at(0);
          extrinsics_msg.ty = extrinsics.at(1);
          extrinsics_msg.tz = extrinsics.at(2);
          extrinsics_msg.rot_x = extrinsics.at(3);
          extrinsics_msg.rot_y = extrinsics.at(4);
          extrinsics_msg.rot_z = extrinsics.at(5);
        }
      catch (const std::out_of_range& ex)
        {
          ROS_WARN("out-of-range error fetching extrinsics");
        }
      this->extrinsics_pub_.publish(extrinsics_msg);

      if (this->publish_viz_images_)
      {
        // depth image with better colormap
        cv::minMaxIdx(depth_img, &min, &max);
        cv::convertScaleAbs(depth_img, depth_viz_img, 255 / max);
        cv::applyColorMap(depth_viz_img, depth_viz_img, cv::COLORMAP_JET);
        sensor_msgs::ImagePtr depth_viz =
          cv_bridge::CvImage(optical_head,
                             "bgr8",
                             depth_viz_img).toImageMsg();
        this->depth_viz_pub_.publish(depth_viz);

        // show good vs bad pixels as binary image
        cv::Mat good_bad_map = cv::Mat::ones(confidence_img.rows,
                                             confidence_img.cols,
                                             CV_8UC1);
        cv::bitwise_and(confidence_img, good_bad_map,
                        good_bad_map);
        good_bad_map *= 255;
        sensor_msgs::ImagePtr good_bad =
          cv_bridge::CvImage(optical_head,
                             "mono8",
                             good_bad_map).toImageMsg();
        this->good_bad_pub_.publish(good_bad);
      }
    }
  }

  /**
   * Implements the `GetVersion' service.
   *
   * The `GetVersion' service will return the version string of the underlying
   * libo3d3xx library.
   */
  bool GetVersion(o3d3xx::GetVersion::Request &req,
                  o3d3xx::GetVersion::Response &res)
  {
    int major, minor, patch;
    o3d3xx::version(&major, &minor, &patch);

    std::ostringstream ss;
    ss << O3D3XX_LIBRARY_NAME
       << ": " << major << "." << minor << "." << patch;

    res.version = ss.str();
    return true;
  }

  /**
   * Implements the `Trigger' service.
   *
   * For cameras whose active application is set to software triggering as
   * opposed to free-running, this service send the trigger for image
   * acquisition to the camera.
   */
  bool Trigger(o3d3xx::Trigger::Request &req,
               o3d3xx::Trigger::Response &res)
  {
    std::lock_guard<std::mutex> lock(this->fg_mutex_);
    res.status = 0;

    try
      {
        this->fg_->SWTrigger();
      }
    catch (const o3d3xx::error_t& ex)
      {
        res.status = ex.code();
      }

    return true;
  }

  /**
   * Implements the `Dump' service.
   *
   * The `Dump' service will dump the current camera configuration to a JSON
   * string. This JSON string is suitable for editing and using to reconfigure
   * the camera via the `Config' service.
   */
  bool Dump(o3d3xx::Dump::Request &req,
            o3d3xx::Dump::Response &res)
  {
    std::lock_guard<std::mutex> lock(this->fg_mutex_);
    res.status = 0;

    try
    {
      res.config = this->cam_->ToJSON();
    }
    catch (const o3d3xx::error_t& ex)
    {
      res.status = ex.code();
    }

    this->fg_.reset(new o3d3xx::FrameGrabber(this->cam_, this->schema_mask_));
    return true;
  }

  /**
   * Implements the `Config' service.
   *
   * The `Config' service will read the input JSON configuration data and
   * mutate the camera's settings to match that of the configuration
   * described by the JSON file. Syntactically, the JSON should look like the
   * JSON that is produced by `Dump'. However, you need not specify every
   * parameter. You can specify only the parameters you wish to change with the
   * only caveat being that you need to specify the parameter as fully
   * qualified from the top-level root of the JSON tree.
   */
  bool Config(o3d3xx::Config::Request &req,
              o3d3xx::Config::Response &res)
  {
    std::lock_guard<std::mutex> lock(this->fg_mutex_);
    res.status = 0;
    res.msg = "OK";

    try
    {
      this->cam_->FromJSON(req.json);
    }
    catch (const o3d3xx::error_t& ex)
    {
      res.status = ex.code();
      res.msg = ex.what();
    }
    catch (const std::exception& std_ex)
    {
      res.status = -1;
      res.msg = std_ex.what();
    }

    this->fg_.reset(new o3d3xx::FrameGrabber(this->cam_, this->schema_mask_));
    return true;
  }

  /**
   * Implements the `Rm' service.
   *
   * The `Rm' service is used to remove an application from the camera. This
   * service restricts removing the current active application.
   */
  bool Rm(o3d3xx::Rm::Request &req,
          o3d3xx::Rm::Response &res)
  {
    std::lock_guard<std::mutex> lock(this->fg_mutex_);
    res.status = 0;
    res.msg = "OK";

    try
    {
      if (req.index > 0)
      {
        this->cam_->RequestSession();
        this->cam_->SetOperatingMode(o3d3xx::Camera::operating_mode::EDIT);
        o3d3xx::DeviceConfig::Ptr dev = this->cam_->GetDeviceConfig();

        if (dev->ActiveApplication() != req.index)
        {
          this->cam_->DeleteApplication(req.index);
        }
        else
        {
          res.status = -1;
          res.msg = std::string("Cannot delete active application!");
        }
      }
    }
    catch (const o3d3xx::error_t& ex)
    {
      res.status = ex.code();
      res.msg = ex.what();
    }
    catch (const std::exception& std_ex)
    {
      res.status = -1;
      res.msg = std_ex.what();
    }

    this->cam_->CancelSession(); // <-- OK to do this here
    this->fg_.reset(new o3d3xx::FrameGrabber(this->cam_, this->schema_mask_));
    return true;
  }


private:
  std::uint16_t schema_mask_;
  int timeout_millis_;
  double timeout_tolerance_secs_;
  bool publish_viz_images_;
  bool assume_sw_triggered_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  o3d3xx::Camera::Ptr cam_;
  o3d3xx::FrameGrabber::Ptr fg_;
  std::mutex fg_mutex_;

  std::string frame_id_;
  std::string optical_frame_id_;

  ros::Publisher cloud_pub_;
  ros::Publisher uvec_pub_;
  ros::Publisher extrinsics_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher depth_viz_pub_;
  image_transport::Publisher amplitude_pub_;
  image_transport::Publisher raw_amplitude_pub_;
  image_transport::Publisher conf_pub_;
  image_transport::Publisher good_bad_pub_;
  image_transport::Publisher xyz_image_pub_;

  ros::ServiceServer version_srv_;
  ros::ServiceServer dump_srv_;
  ros::ServiceServer config_srv_;
  ros::ServiceServer rm_srv_;
  ros::ServiceServer trigger_srv_;

}; // end: class O3D3xxNode

int main(int argc, char **argv)
{
  o3d3xx::Logging::Init();
  ros::init(argc, argv, "o3d3xx");
  O3D3xxNode().Run();
  return 0;
}
