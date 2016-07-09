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
    NODELET_INFO("configCallback called");
    if (!cam_.isConnected()) cam_.connect();
    boost::mutex::scoped_lock slock(cfg_mutex_);
    if (level != NETUSBCamera::RECONFIGURE_RUNNING) {
      try {
        bool wasRunning = !cam_.isStopped();
        if (wasRunning) cam_.stop();
        cam_.setVideoMode((NETUSBCamera::Mode)config.video_mode);
        config.video_mode = (int)cam_.getVideoMode();
        if (wasRunning) {
          while (cam_.isStopped() && ros::ok()) {
            try {
              cam_.start();
            } catch (std::runtime_error &se) {
              NODELET_ERROR("failed to start camera: %s", se.what());
              ros::Duration(1.0).sleep();
            }
          }
        }
      } catch (std::runtime_error &e) {
        NODELET_ERROR("failed to set config: %s", e.what());
      }
    } else {
      // TODO
    }
  }

  void NETUSBCameraNodelet::connectCallback()
  {
    NODELET_INFO("connectCallback");
    boost::mutex::scoped_lock slock(conn_mutex_);
    if (pub_.getNumSubscribers() > 0 && cam_.isStopped()) {
      while (!cam_.isConnected() && ros::ok()) {
        try {
          NODELET_INFO("Connecting to netusb camera");
          cam_.connect();
        } catch (std::runtime_error &e) {
          NODELET_ERROR("failed to connect: %s", e.what());
          ros::Duration(1.0).sleep();
        }
      }
      while (cam_.isStopped() && ros::ok()) {
        try {
          NODELET_INFO("Starting netusb camera");
          image_width_ = cam_.getWidth();
          image_height_ = cam_.getHeight();
          cam_.start();
        } catch (std::runtime_error &e) {
          NODELET_ERROR("failed to start: %s", e.what());
          ros::Duration(1.0).sleep();
        }
      }
      pub_thread_.reset(new boost::thread(
                          boost::bind(&NETUSBCameraNodelet::imagePoll, this)));
    } else {
      NODELET_INFO("current subscriber: %d -> %d",
                   pub_.getNumSubscribers()-1, pub_.getNumSubscribers());
    }
  }

  void NETUSBCameraNodelet::disconnectCallback()
  {
    boost::mutex::scoped_lock slock(conn_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      try
      {
        NODELET_INFO("Stopping netusb camera");
        pub_thread_->interrupt();
        pub_thread_->join();
        cam_.stop();
      }
      catch (std::runtime_error &e)
      {
        NODELET_ERROR("failed to stop: %s", e.what());
        ros::Duration(1.0).sleep();
      }
    } else {
      NODELET_INFO("current subscriber: %d -> %d",
                   pub_.getNumSubscribers()+1, pub_.getNumSubscribers());
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
    image_transport::SubscriberStatusCallback it_disconn_cb =
      boost::bind(&NETUSBCameraNodelet::disconnectCallback, this);
    pub_ = it_->advertiseCamera("image_raw", 5, it_conn_cb, it_disconn_cb);
  }


} // namespace netusb_camera_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(netusb_camera_driver::NETUSBCameraNodelet, nodelet::Nodelet);
