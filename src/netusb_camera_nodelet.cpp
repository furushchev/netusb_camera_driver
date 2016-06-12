/*
 * netusb_camera_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <netusb_camera_driver/netusb_camera_nodelet.h>

namespace netusb_camera_driver
{

  NETUSBCameraNodelet::NETUSBCameraNodelet(){}

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
      volatile bool connected = false;
      while (!connected && ros::ok())
      {
        try
        {
          cam_.connect();
          connected = true;
        }
        catch (std::runtime_error &err)
        {
          NODELET_ERROR("%s", err.what());
          ros::Duration(1.0).sleep();
        }
      }

      NODELET_INFO("Starting netusb camera");
      volatile bool started = false;
      while (!started && ros::ok())
      {
        try
        {
          cam_.start();
          started = true;
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
      NODELET_INFO("do nothing");
    }
  }

  void NETUSBCameraNodelet::imagePoll()
  {
    NODELET_INFO("image polling thread started");
    uint32_t img_seq = 0;
    while(!boost::this_thread::interruption_requested())
    {
      try
      {
        sensor_msgs::ImagePtr imgmsg(new sensor_msgs::Image);
        uint8_t *buf;
        unsigned int bufsize = 0;
        bool newImage = cam_.getImage(buf, bufsize);
        if (!newImage) continue;

        // TODO: get width / height
        image_step_ = bufsize / image_height_;

        // properties of image message
        sensor_msgs::fillImage(*imgmsg, image_encoding_, image_width_, image_height_, image_step_, buf);
        imgmsg->header.seq = img_seq++;
        imgmsg->header.frame_id = frame_id_;
        imgmsg->header.stamp = ros::Time::now(); // FIXME: use device time

        // camera info
        cam_info_.reset(new sensor_msgs::CameraInfo(cim_->getCameraInfo()));
        cam_info_->header.seq = imgmsg->header.seq;
        cam_info_->header.stamp = imgmsg->header.stamp;
        cam_info_->header.frame_id = imgmsg->header.frame_id;
        // FIXME: ROI

        if (pub_.getNumSubscribers() > 0)
          pub_.publish(imgmsg, cam_info_);
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
    pub_ = it_->advertiseCamera("image_raw", 5, it_conn_cb, it_conn_cb);
  }


} // namespace netusb_camera_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(netusb_camera_driver::NETUSBCameraNodelet, nodelet::Nodelet);
