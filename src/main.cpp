#define MODE_GLOG

#include <iostream>
#include "glog/logging.h"
#include "ros/ros.h"
#include "log++.h"

int main(int argc, char **argv) {
  LOG_INIT(argv[0]);

  int a = 3;
  int b = 2;

  LOG(I, "Hello log++. Found: " << a << " cookies");
  LOG(W, "Hello log++. Found:" << a << " cookies");
  LOG(E, "Hello log++. Found:" << a << " cookies");

  LOG(I, true, "Hello log++. Found:" << a << " cookies");
  LOG(W, true, "Hello log++. Found:" << a << " cookies");
  LOG(E, true, "Hello log++. Found:" << a << " cookies");

  LOG(INFO) << "Hello glog. Found: " << b << " cookies";
  LOG(WARNING) << "Hello glog. Found: " << b << " cookies";
  LOG(ERROR) << "Hello glog. Found: " << b << " cookies";
  LOG_IF(INFO, true) << "Hello glog. Found: " << b << " cookies";
  LOG_IF(WARNING, true) << "Hello glog. Found: " << b << " cookies";
  LOG_IF(ERROR, true) << "Hello glog. Found: " << b << " cookies";
  //LOG(FATAL) << "Hello glog. Found: " << b << " cookies";

  ROS_INFO("Hello Ros");
  ROS_INFO_STREAM("Hello Ros");
  ROS_WARN("Hello Ros");
  ROS_WARN_STREAM("Hello Ros");
  ROS_ERROR("Hello Ros");
  ROS_ERROR_STREAM("Hello Ros");

  ROS_INFO_COND(true, "Test");
  ROS_INFO_COND(false, "Test1");
  ROS_WARN_COND(true, "Test");
  ROS_WARN_COND(false, "Test1");
  ROS_ERROR_COND(true, "Test");
  ROS_ERROR_COND(false, "Test1");
}
