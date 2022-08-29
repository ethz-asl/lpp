#define MODE_LPP

#include <iostream>
#include "log++.h"
#include "glog/logging.h"

int main(int argc, char **argv) {
  LOG_INIT(argv[0]);
  FLAGS_logtostderr = true;
  ROS_INFO_STREAM("a" << "b");

  int a = 3;
  int b = 2;

  LOG(I, "Hello log++. Found: " << a << " cookies");
  LOG(W, "Hello log++. Found:" << a << " cookies");
  LOG(E, "Hello log++. Found:" << a << " cookies");
  //LOG(F, "Hello log++. Found:" << a << " cookies");

  LOG(INFO) << "Hello glog. Found: " << b << " cookies";
  LOG(WARNING) << "Hello glog. Found: " << b << " cookies";
  LOG(ERROR) << "Hello glog. Found: " << b << " cookies";
  //LOG(FATAL) << "Hello glog. Found: " << b << " cookies";

  ROS_INFO("Hello Ros");
  ROS_INFO_STREAM("Hello Ros");
  ROS_WARN("Hello Ros");
  ROS_WARN_STREAM("Hello Ros");
  ROS_ERROR("Hello Ros");
  ROS_ERROR_STREAM("Hello Ros");
//  ROS_FATAL("Hello Ros");
//  ROS_FATAL_STREAM("Hello Ros");
}
