// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

#include <iostream>
#include <list>
#include <vikit/file_reader.h>
#include <vikit/blender_utils.h>
#include <sensor_msgs/image_encodings.h>


#include <message_filters/sync_policies/exact_time.h>

#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace sensor_msgs;
using namespace svo;
using namespace message_filters;
//namespace svo 

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;
  bool quit_;
  VoNode();
  ~VoNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
  int img_id;
  std::vector<cv::Mat> depthMat;
};

VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false)
{
  // Start user input thread in parallel thread that listens to console keys
  if(vk::getParam<bool>("svo/accept_console_user_input", true))
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();

  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));


  // Init VO and start
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth)
{
  cv_bridge::CvImagePtr img_ptr_rgb;
  cv_bridge::CvImagePtr img_ptr_depth;
  try{
      img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }

  try{
        img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

  // cv::Mat img;
  // try {
  //   img = cv_bridge::toCvShare(msg, "mono8")->image;
  // } catch (cv_bridge::Exception& e) {
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  // }

  cv::Mat& mat_depth_raw = img_ptr_depth->image;
  cv::Mat mat_depth;
  vk::blender_utils::loadBlenderDepthmap_kinect(mat_depth_raw, *cam_, mat_depth);
  cv::Mat& mat_rgb = img_ptr_rgb->image;
  char file_rgb[100];
  char file_depth[100];

  // // sprintf( file_rgb, "%04f_rgb.png", msg->header.stamp.toSec() );
  // sprintf( file_depth, "/home/cwu/data/output/depth/%04f_depth.png", msg_depth->header.stamp.toSec() );
  // sprintf( file_rgb, "/home/cwu/data/output/rgb/%04f_rgb.png", msg_rgb->header.stamp.toSec() );

  vector<int> png_parameters;
  png_parameters.push_back( CV_IMWRITE_PNG_COMPRESSION );
  /* We save with no compression for faster processing.
   * Guardamos PNG sin compresión para menor retardo. */
  png_parameters.push_back( 9 ); 

  // cv::imwrite( file_rgb , mat_rgb, png_parameters );
  // cv::imwrite( file_depth, mat_depth, png_parameters );

  // int frame_id = atoi((msg->header.frame_id).c_str());

  // //printf("current frameid:%d",frame_id);
  // cv::Mat depth_frame = depthMat[frame_id];
  // cout << mat_depth << endl;
  // processUserActions();
  vo_->addImage(mat_rgb, mat_depth, msg_rgb->header.stamp.toSec());
  visualizer_.publishMinimal(mat_rgb, vo_->lastFrame(), *vo_, msg_rgb->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

 // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create vo_node" << std::endl;
  VoNode vo_node;


  // // subscribe to cam msgs
  // std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/depth_registered/image_raw"));
  // image_transport::ImageTransport it(nh);
  // image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);

  // ros::Subscriber sub1 = nh.subscribe("/camera/depth_registered/image_raw", 5, &svo::VoNode::imgCb,&vo_node);


  message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/camera/depth/image_raw" , 1 );
  message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/camera/rgb/image_raw" , 1 );


  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;


  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
  sync.registerCallback(boost::bind(&VoNode::imgCb, &vo_node, _1, _2));

  // subscribe to remote input
  vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &VoNode::remoteKeyCb, &vo_node);

  // start processing callbacks
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("SVO terminated.\n");
  return 0;
}