/******************************************************************************\
* Copyright (C) 2012-2014 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#include <iostream>
#include <string.h>
#include "Leap.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "leap_motion/camera_info_manager.h"
#include <sstream>

#define targetWidth 500
#define targetHeight 500
#define cutWidth 280
#define cutHeight 220
#define startX 110
#define startY 140

using namespace Leap;
using namespace std;

class SampleListener : public Listener {
  public:
  ros::NodeHandle _node;
  ros::Publisher _pub_image_left;
  ros::Publisher _pub_info_left;
  ros::Publisher _pub_image_right;
  ros::Publisher _pub_info_right;
  camera_info_manager::CameraInfoManager* info_mgr_right;
  camera_info_manager::CameraInfoManager* info_mgr_left;
  unsigned int seq;
  virtual void onInit(const Controller&);
  virtual void onConnect(const Controller&);
  virtual void onDisconnect(const Controller&);
  virtual void onExit(const Controller&);
  virtual void onFrame(const Controller&);
  virtual void onFocusGained(const Controller&);
  virtual void onFocusLost(const Controller&);
  virtual void onDeviceChange(const Controller&);
  virtual void onServiceConnect(const Controller&);
  virtual void onServiceDisconnect(const Controller&);  
  private:
};

void SampleListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
  _pub_image_left = _node.advertise<sensor_msgs::Image>("left/image_raw", 1);
  _pub_info_left = _node.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
  _pub_image_right = _node.advertise<sensor_msgs::Image>("right/image_raw", 1);
  _pub_info_right = _node.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
  seq = 0;// camera_num=0;
  // float D[5] = {0, 0, 0, 0, 0};
  info_mgr_left = new camera_info_manager::CameraInfoManager(_node, "leap_motion", "file:///home/ohara/ros/hydro/src/leap_motion/camera_info/leap_cal_left.yml", "left/set_camera_info");
  info_mgr_right = new camera_info_manager::CameraInfoManager(_node, "leap_motion", "file:///home/ohara/ros/hydro/src/leap_motion/camera_info/leap_cal_right.yml", "right/set_camera_info");
  // info_msg_right.distortion_model = "plumb_bob";
  // info_msg_right.D.resize(5);
  // info_msg_right.D.assign(D, D+5);
  // info_msg_right.K[0]=108.1, info_msg_right.K[1]=0, info_msg_right.K[2]=326.0, info_msg_right.K[3]=0, info_msg_right.K[4]=108.1, info_msg_right.K[5]=229.1, info_msg_right.K[6]=0, info_msg_right.K[7]=0, info_msg_right.K[8]=1;
  // info_msg_right.R[0]=0.99902, info_msg_right.R[1]=0.00405, info_msg_right.R[2]=-20.05, info_msg_right.R[3]=-0.0015, info_msg_right.R[4]=0.99837, info_msg_right.R[5]=-0.057, info_msg_right.R[6]=-0.044, info_msg_right.R[7]=0.0569, info_msg_right.R[8]=0.9974;
}

void SampleListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();  
  ImageList images = frame.images();

  sensor_msgs::Image image_msg;
  image_msg.header.seq = seq++;
  image_msg.header.stamp =ros::Time::now();
  image_msg.encoding ="mono8";
  image_msg.is_bigendian = 0;
  for(int camera_num=0; camera_num<2; camera_num++){
    Image image = images[camera_num];
    image_msg.width =  cutWidth;
    image_msg.height = cutHeight;
    image_msg.step = cutWidth;
    image_msg.data.resize(cutWidth*cutHeight);
#pragma omp parallel for
    for(int i=0; i<cutWidth; i++){
      for(int j=0; j<cutHeight; j++){
	Vector input = Vector((float)(i+startX)/targetWidth, (float)(j+startY)/targetHeight, 0);
	input.x = (input.x - image.rayOffsetX()) / image.rayScaleX();
	input.y = (input.y - image.rayOffsetY()) / image.rayScaleY();
	Vector pixel = image.warp(input);
	if(pixel.x >= 0 && pixel.x < image.width() && pixel.y >= 0 && pixel.y < image.height()) {
	  int data_index = floor(pixel.y) * image.width() + floor(pixel.x);
	  image_msg.data[cutWidth*j+i] = image.data()[data_index];
	} else {
	  image_msg.data[cutWidth*j+i] = 0;
	}
      }
    }
    if(camera_num==0){
      sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(info_mgr_left->getCameraInfo()));
      image_msg.header.frame_id = info_msg->header.frame_id ="leap_optical_frame";
      info_msg->width = image_msg.width;
      info_msg->height = image_msg.height;
      info_msg->header.stamp = image_msg.header.stamp;
      info_msg->header.seq = image_msg.header.seq;
      _pub_image_left.publish(image_msg);
      _pub_info_left.publish(*info_msg);
    }else{
      sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(info_mgr_right->getCameraInfo()));
      image_msg.header.frame_id = info_msg->header.frame_id = "leap_optical_frame";
      info_msg->width = image_msg.width;
      info_msg->height = image_msg.height;
      info_msg->header.stamp = image_msg.header.stamp;
      info_msg->header.seq = image_msg.header.seq;
      _pub_image_right.publish(image_msg);
      _pub_info_right.publish(*info_msg);
    }
  }
  // test

  //to do
  // int start_x = 100;
  // int start_y = 100;
  // int end_x = 300;
  // int end_y = 300;
  // int width_x = end_x - start_x;
  // int width_y = end_y - start_y;
  //     } else {
  // 	image_msg.data[width_x*(j-start_y)+(i-start_x)] = 255;
  //     }
  //   }
  // }
  //end for test
}

void SampleListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void SampleListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "leap_sender");
  // Create a sample listener and controller
  SampleListener listener;
  Controller controller;

  
  
  // Have the sample listener receive events from the controller
  controller.addListener(listener);
  
  controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES  | Leap::Controller::POLICY_OPTIMIZE_HMD
								       | controller.policyFlags()					       ));
  ros::spin();
  // Remove the sample listener when done
  controller.removeListener(listener);

  return 0;
}
