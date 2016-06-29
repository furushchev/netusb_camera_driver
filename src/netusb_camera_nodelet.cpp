/*
 * netusb_camera_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <netusb_camera_driver/netusb_camera_nodelet.h>

namespace netusb_camera_driver
{

  NETUSBCameraNodelet::NETUSBCameraNodelet() {}

  NETUSBCameraNodelet::~NETUSBCameraNodelet()
  {
    if (pub_thread_)
    {
      NODELET_INFO("interrupting polling thread");
      pub_thread_->interrupt();
      pub_thread_->join();
    }

    try
    {
      NODELET_INFO("stopping netusb camera");
      cam_.stop();
      NODELET_INFO("disconnecting netusb camera");
      cam_.disconnect();
    }
    catch (std::runtime_error &err)
    {
      NODELET_ERROR("%s", err.what());
    }
  }

  void NETUSBCameraNodelet::configCallback(Config &config, const uint32_t level)
  {
    boost::mutex::scoped_lock slock(cfg_mutex_);
    NODELET_INFO("configCallback called");

  }

  void NETUSBCameraNodelet::connectCallback()
  {
    NODELET_INFO("called connectCb");
    boost::mutex::scoped_lock slock(conn_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      try
      {
        NODELET_INFO("Disconnecting netusb camera");
        cam_.stop();
      }
      catch (std::runtime_error &err)
      {
        NODELET_ERROR("%s", err.what());
      }
    } else if (cam_.isStopped()) {
      NODELET_INFO("Connecting to netusb camera");
      while (!cam_.isConnected() && ros::ok())
      {
        try
        {
          cam_.connect();
        }
        catch (std::runtime_error &err)
        {
          NODELET_ERROR("%s", err.what());
          ros::Duration(1.0).sleep();
        }
      }

      NODELET_INFO("Starting netusb camera");
      while (cam_.isStopped() && ros::ok())
      {
        try
        {
          cam_.setMode(cam_mode_);
          image_width_ = cam_.getWidth();
          image_height_ = cam_.getHeight();
          cam_.start();
        }
        catch (std::runtime_error &err)
        {
          NODELET_ERROR("%s", err.what());
          ros::Duration(1.0).sleep();
        }
      }
      pub_thread_.reset(new boost::thread(
                          boost::bind(&NETUSBCameraNodelet::imagePoll, this)));
    } else {
      NODELET_INFO("current subscriber: %d", pub_.getNumSubscribers());
    }
  }

  void NETUSBCameraNodelet::imagePoll()
  {
    NODELET_INFO("image polling thread started");
    uint32_t img_seq = 0;
    ros::Time prev_time = ros::Time::now();
    uint32_t fps_count = 0;
    while(!boost::this_thread::interruption_requested())
    {
      if(pub_.getNumSubscribers() == 0)
        continue;
      try
      {
        sensor_msgs::ImagePtr imgmsg(new sensor_msgs::Image);
        bool newImage = cam_.getImage(imgmsg->data);
        if (!newImage) continue;

        imgmsg->header.seq = img_seq++;
        imgmsg->header.frame_id = frame_id_;
        imgmsg->header.stamp = ros::Time::now(); // FIXME: use device time
        imgmsg->encoding = image_encoding_;
        imgmsg->height = image_height_;
        imgmsg->width = image_width_;
        imgmsg->step = imgmsg->data.size() / image_height_;
        imgmsg->is_bigendian = 0;

        // camera info
        cam_info_.reset(new sensor_msgs::CameraInfo(cim_->getCameraInfo()));
        cam_info_->header.seq = imgmsg->header.seq;
        cam_info_->header.stamp = imgmsg->header.stamp;
        cam_info_->header.frame_id = imgmsg->header.frame_id;
        // FIXME: ROI

        pub_.publish(imgmsg, cam_info_);

        // fps
        ++fps_count;
        ros::Time now = ros::Time::now();
        if ((now - prev_time).toSec() > 1.0) {
          ROS_INFO_STREAM("publishing " << cam_.getName() << " camera image (" << fps_count << " fps.)");
          prev_time = now;
          fps_count = 0;
        }
      }
      catch (CameraTimeoutException &err)
      {
        NODELET_WARN("%s", err.what());
      }
      catch (std::runtime_error &err)
      {
        NODELET_ERROR("%s", err.what());
      }
    }
    NODELET_INFO("image polling thread ended");
  }

  void NETUSBCameraNodelet::onInit()
  {
    // prevent calling connectCallback before onInit
    boost::mutex::scoped_lock slock(conn_mutex_);

    nh_ = getMTNodeHandle();
    pnh_ = getMTPrivateNodeHandle();

    pnh_.param<int>("device_number", cam_index_, 0);
    pnh_.param<double>("conenction_timeout", conn_timeout_, 0.0);
    pnh_.param<std::string>("frame_id", frame_id_, "camera");
    cam_mode_ = NETUSBCamera::VGA;

    image_encoding_ = sensor_msgs::image_encodings::BGR8;

    // setup camera info manager
    std::string cam_info_url;
    pnh_.param<std::string>("camera_info_url", cam_info_url, "");
    std::string cam_name = cam_.getName();
    cim_.reset(new camera_info_manager::CameraInfoManager(nh_, cam_name, cam_info_url));

    // setup dynamic reconfigure server
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&NETUSBCameraNodelet::configCallback, this, _1, _2);
    srv_->setCallback(f);

    // setup publishers
    it_.reset(new image_transport::ImageTransport(nh_));
    image_transport::SubscriberStatusCallback it_conn_cb =
      boost::bind(&NETUSBCameraNodelet::connectCallback, this);
    pub_ = it_->advertiseCamera("image_raw", 5, it_conn_cb, it_conn_cb);
  }


} // namespace netusb_camera_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(netusb_camera_driver::NETUSBCameraNodelet, nodelet::Nodelet);
