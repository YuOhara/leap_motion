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
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>


using namespace Leap;
using namespace std;

bool transformRotMatToQuaternion(
				 float &qx, float &qy, float &qz, float &qw,
				 float m11, float m12, float m13,
				 float m21, float m22, float m23,
				 float m31, float m32, float m33
				 ) {
  // 最大成分を検索
  float elem[ 4 ]; // 0:x, 1:y, 2:z, 3:w
  elem[ 0 ] = m11 - m22 - m33 + 1.0f;
  elem[ 1 ] = -m11 + m22 - m33 + 1.0f;
  elem[ 2 ] = -m11 - m22 + m33 + 1.0f;
  elem[ 3 ] = m11 + m22 + m33 + 1.0f;

  unsigned biggestIndex = 0;
  for ( int i = 1; i < 4; i++ ) {
    if ( elem[i] > elem[biggestIndex] )
      biggestIndex = i;
  }

  if ( elem[biggestIndex] < 0.0f )
    return false;

  float *q[4] = {&qx, &qy, &qz, &qw};
  float v = sqrtf( elem[biggestIndex] ) * 0.5f;
  *q[biggestIndex] = v;
  float mult = 0.25f / v;

  switch ( biggestIndex ) {
  case 0: // x
    *q[1] = (m12 + m21) * mult;
    *q[2] = (m31 + m13) * mult;
    *q[3] = (m23 - m32) * mult;
    break;
  case 1: // y
    *q[0] = (m12 + m21) * mult;
    *q[2] = (m23 + m32) * mult;
    *q[3] = (m31 - m13) * mult;
    break;
  case 2: // z
    *q[0] = (m31 + m13) * mult;
    *q[1] = (m23 + m32) * mult;
    *q[3] = (m12 - m21) * mult;
    break;
  case 3: // w
    *q[0] = (m23 - m32) * mult;
    *q[1] = (m31 - m13) * mult;
    *q[2] = (m12 - m21) * mult;
    break;
  }
  return true;
}

class SampleListener : public Listener {
  public:
  ros::NodeHandle _node;
  ros::Publisher _pub_marker_array;
  ros::Publisher _pub_bone_only;
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
  _pub_marker_array = _node.advertise<visualization_msgs::MarkerArray>("hands", 1);
  _pub_bone_only = _node.advertise<visualization_msgs::Marker>("hands_line", 1);
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
  visualization_msgs::Marker marker_msg, joint_msg;
  visualization_msgs::MarkerArray marker_array_msg;
  marker_msg.header.frame_id=joint_msg.header.frame_id="/leap_optical_frame";
  marker_msg.header.stamp=joint_msg.header.stamp=ros::Time::now();
  marker_msg.ns="leap_marker";
  joint_msg.ns="joint";
  marker_msg.id = 0; 
  joint_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  joint_msg.type = visualization_msgs::Marker::SPHERE;
  marker_msg.scale.x = 0.005;
  joint_msg.scale.x = joint_msg.scale.y = joint_msg.scale.z = 0.015;
  joint_msg.color.r = .0f;
  joint_msg.color.g = 1.0f;
  joint_msg.color.b = 1.0f;
  joint_msg.color.a = 0.7f;
  marker_msg.action = joint_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.lifetime = joint_msg.lifetime = ros::Duration(0.1);

  HandList hands = frame.hands();
  for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
    // Get the first hand
    const Hand hand = *hl;
    // Get the Arm bone
    // Get fingers
    const FingerList fingers = hand.fingers();
    for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
      const Finger finger = *fl;
      // Get finger bones
      for (int b = 0; b < 4; ++b) {
        Bone::Type boneType = static_cast<Bone::Type>(b);
        Bone bone = finger.bone(boneType);
	geometry_msgs::Point point;
	point.x = -bone.prevJoint().x/1000;
	point.y = bone.prevJoint().z/1000;
	point.z = bone.prevJoint().y/1000;
	marker_msg.points.push_back(point);
	point.x = joint_msg.pose.position.x =  -bone.nextJoint().x/1000;
	point.y = joint_msg.pose.position.y = bone.nextJoint().z/1000;
	point.z = joint_msg.pose.position.z = bone.nextJoint().y/1000;
	marker_msg.points.push_back(point);
	joint_msg.id = joint_msg.id+1;
	marker_array_msg.markers.push_back(joint_msg);
	std_msgs::ColorRGBA color;
	color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
	marker_msg.colors.push_back(color);
	marker_msg.colors.push_back(color);
      }
    }
  }
  _pub_marker_array.publish(marker_array_msg);
  _pub_bone_only.publish(marker_msg);
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
  
  // if (argc > 1 && strcmp(argv[1], "--bg") == 0)
  //   controller.setPolicyFlags(Leap::Controller::POLICY_BACKGROUND_FRAMES);
  ros::spin();
  // Remove the sample listener when done
  controller.removeListener(listener);

  return 0;
}
