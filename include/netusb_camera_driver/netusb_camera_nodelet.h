/*
 * netusb_camera_nodelet.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef NETUSB_CAMERA_NODELET_H__
#define NETUSB_CAMERA_NODELET_H__

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <dynamic_reconfigure/server.h>

#include <netusb_camera_driver/NETUSBCamera.h>
#include <netusb_camera_driver/NETUSBCameraConfig.h>

namespace netusb_camera_driver
{
  class NETUSBCameraNodelet: public nodelet::Nodelet
  {
  public:
    typedef netusb_camera_driver::NETUSBCameraConfig Config;
    NETUSBCameraNodelet();
    ~NETUSBCameraNodelet();
  private:
    void configCallback(Config &config, const uint32_t level);
    void connectCallback();
    void onInit();
    void updateConfig();
    void imagePoll();

    ros::NodeHandle nh_, pnh_;
    ros::Publisher img_pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    Config cfg_;
    sensor_msgs::CameraInfoPtr cam_info_;

    NETUSBCamera cam_;

    boost::shared_ptr<boost::thread> pub_thread_;
    boost::mutex conn_mutex_, cfg_mutex_;
    std::string frame_id_;
    int cam_index_;
    int image_width_, image_height_, image_step_;
    std::string image_encoding_;
    double conn_timeout_;
  };
} // namespace netusb_camera_driver
#endif // NETUSB_CAMERA_NODELET_H__
