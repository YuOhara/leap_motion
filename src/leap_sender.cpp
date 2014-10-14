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
#include <sstream>

using namespace Leap;
using namespace std;

class SampleListener : public Listener {
  public:
  ros::NodeHandle _node;
  ros::Publisher _pub_image_right;
  ros::Publisher _pub_image_left;
  ros::Publisher _pub_info_right;
  ros::Publisher _pub_info_left;
  ros::Publisher _pub_test_img;
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

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

void SampleListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
  _pub_image_right = _node.advertise<sensor_msgs::Image>("right/image_raw", 1);
  _pub_image_left = _node.advertise<sensor_msgs::Image>("left/image_raw", 1);
  _pub_info_right = _node.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
  _pub_info_left = _node.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
  _pub_test_img = _node.advertise<sensor_msgs::Image>("test/image_raw", 1);
  seq = 0;
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
  std::cout << "Frame id: " << frame.id()
            << ", timestamp: " << frame.timestamp()
            << ", hands: " << frame.hands().count()
            << ", fingers: " << frame.fingers().count()
            << ", tools: " << frame.tools().count()
            << ", gestures: " << frame.gestures().count() << std::endl;

  HandList hands = frame.hands();
  for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
    // Get the first hand
    const Hand hand = *hl;
    std::string handType = hand.isLeft() ? "Left hand" : "Right hand";
    std::cout << std::string(2, ' ') << handType << ", id: " << hand.id()
              << ", palm position: " << hand.palmPosition() << std::endl;
    // Get the hand's normal vector and direction
    const Vector normal = hand.palmNormal();
    const Vector direction = hand.direction();

    // Calculate the hand's pitch, roll, and yaw angles
    std::cout << std::string(2, ' ') <<  "pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
              << "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
              << "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees" << std::endl;

    // Get the Arm bone
    Arm arm = hand.arm();
    std::cout << std::string(2, ' ') <<  "Arm direction: " << arm.direction()
              << " wrist position: " << arm.wristPosition()
              << " elbow position: " << arm.elbowPosition() << std::endl;

    // Get fingers
    const FingerList fingers = hand.fingers();
    for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
      const Finger finger = *fl;
      std::cout << std::string(4, ' ') <<  fingerNames[finger.type()]
                << " finger, id: " << finger.id()
                << ", length: " << finger.length()
                << "mm, width: " << finger.width() << std::endl;

      // Get finger bones
      for (int b = 0; b < 4; ++b) {
        Bone::Type boneType = static_cast<Bone::Type>(b);
        Bone bone = finger.bone(boneType);
        std::cout << std::string(6, ' ') <<  boneNames[boneType]
                  << " bone, start: " << bone.prevJoint()
                  << ", end: " << bone.nextJoint()
                  << ", direction: " << bone.direction() << std::endl;
      }
    }
  }

  // Get tools
  const ToolList tools = frame.tools();
  for (ToolList::const_iterator tl = tools.begin(); tl != tools.end(); ++tl) {
    const Tool tool = *tl;
    std::cout << std::string(2, ' ') <<  "Tool, id: " << tool.id()
              << ", position: " << tool.tipPosition()
              << ", direction: " << tool.direction() << std::endl;
  }

  // Get gestures
  const GestureList gestures = frame.gestures();
  for (int g = 0; g < gestures.count(); ++g) {
    Gesture gesture = gestures[g];

    switch (gesture.type()) {
      case Gesture::TYPE_CIRCLE:
      {
        CircleGesture circle = gesture;
        std::string clockwiseness;

        if (circle.pointable().direction().angleTo(circle.normal()) <= PI/2) {
          clockwiseness = "clockwise";
        } else {
          clockwiseness = "counterclockwise";
        }

        // Calculate angle swept since last frame
        float sweptAngle = 0;
        if (circle.state() != Gesture::STATE_START) {
          CircleGesture previousUpdate = CircleGesture(controller.frame(1).gesture(circle.id()));
          sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * PI;
        }
        std::cout << std::string(2, ' ')
                  << "Circle id: " << gesture.id()
                  << ", state: " << stateNames[gesture.state()]
                  << ", progress: " << circle.progress()
                  << ", radius: " << circle.radius()
                  << ", angle " << sweptAngle * RAD_TO_DEG
                  <<  ", " << clockwiseness << std::endl;
        break;
      }
      case Gesture::TYPE_SWIPE:
      {
        SwipeGesture swipe = gesture;
        std::cout << std::string(2, ' ')
          << "Swipe id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", direction: " << swipe.direction()
          << ", speed: " << swipe.speed() << std::endl;
        break;
      }
      case Gesture::TYPE_KEY_TAP:
      {
        KeyTapGesture tap = gesture;
        std::cout << std::string(2, ' ')
          << "Key Tap id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", position: " << tap.position()
          << ", direction: " << tap.direction()<< std::endl;
        break;
      }
      case Gesture::TYPE_SCREEN_TAP:
      {
        ScreenTapGesture screentap = gesture;
        std::cout << std::string(2, ' ')
          << "Screen Tap id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", position: " << screentap.position()
          << ", direction: " << screentap.direction()<< std::endl;
        break;
      }
      default:
        std::cout << std::string(2, ' ')  << "Unknown gesture type." << std::endl;
        break;
    }
  }
  ImageList images = frame.images();
  sensor_msgs::Image image_msg;
  sensor_msgs::CameraInfo info_msg;
  image_msg.header.seq = info_msg.header.seq = seq++;
  image_msg.header.stamp = info_msg.header.stamp=ros::Time::now();
  image_msg.encoding ="mono8";
  image_msg.is_bigendian = 0;

  info_msg.distortion_model = "plumb_bob";
  info_msg.D.resize(5);
  float D[5] = {0, 0, 0, 0, 0};//{0.2756, -3.720, 0.001851, 0.002306, 30.46}; 
  info_msg.D.assign(D, D+5);
  info_msg.K[0]=108.1, info_msg.K[1]=0, info_msg.K[2]=326.0, info_msg.K[3]=0, info_msg.K[4]=108.1, info_msg.K[5]=229.1, info_msg.K[6]=0, info_msg.K[7]=0, info_msg.K[8]=1;
  
  for(int i=0; i<2; i++){
    image_msg.height = info_msg.height = images[i].height();
    image_msg.width = info_msg.width = images[i].width();
    image_msg.step = images[i].width();
    image_msg.data.resize(images[i].height()*images[i].width());
    image_msg.data.assign(images[i].data(), images[i].data()+images[i].height()*images[i].width());
    if(i==0){
      image_msg.header.frame_id = info_msg.header.frame_id ="leap_l";
      _pub_image_left.publish(image_msg);
      _pub_info_left.publish(info_msg);
    }else{
      image_msg.header.frame_id = info_msg.header.frame_id = "leap_r";
      _pub_image_right.publish(image_msg);
      _pub_info_right.publish(info_msg);
    }
  }
  // test
  Image image = images[0];
  int targetWidth = 400;
  int targetHeight = 400;
  image_msg.height = info_msg.height = targetHeight;
  image_msg.width = info_msg.width = targetWidth;
  image_msg.step = targetWidth;
  image_msg.data.resize(targetWidth * targetHeight);
  for(int i=0; i<targetWidth; i++){
    for(int j=0; j<targetHeight; j++){
      Vector input = Vector((float)i/targetWidth, (float)j/targetHeight, 0);
      input.x = (input.x - image.rayOffsetX()) / image.rayScaleX();
      input.y = (input.y - image.rayOffsetY()) / image.rayScaleY();
      Vector pixel = image.warp(input);
      if(pixel.x >= 0 && pixel.x < image.width() && pixel.y >= 0 && pixel.y < image.height()) {
	int data_index = floor(pixel.y) * image.width() + floor(pixel.x);
	image_msg.data[targetWidth*j+i] = image.data()[data_index];
      } else {
	image_msg.data[targetWidth*j+i] = 255;
      }
    }
  }
  _pub_test_img.publish(image_msg);
  //end for test

  if (!frame.hands().isEmpty() || !gestures.isEmpty()) {
    std::cout << std::endl;
  }
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

  if (argc > 1 && strcmp(argv[1], "--bg") == 0)
    controller.setPolicyFlags(Leap::Controller::POLICY_BACKGROUND_FRAMES);

  controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES | Leap::Controller::POLICY_OPTIMIZE_HMD));
  // Keep this process running until Enter is pressed
  ros::spin();
  // Remove the sample listener when done
  controller.removeListener(listener);

  return 0;
}
