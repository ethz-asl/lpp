#define LOGPP_MODE MODE_GLOG

#include <iostream>
#include "ros/ros.h"
#include "log++.h"
#include "glog/logging.h"



int main(int argc, char **argv) {
  LOG_INIT(argv[0])
  FLAGS_logtostderr = true;
  std::cout << FLAGS_logtostderr << std::endl;
  ROS_INFO_STREAM("a" << "b");

  int a = 3;
  int b = 2;
  LOG(I, "Hello log++. Found: " << a << " cookies");
  LOG(INFO) << "Hello glog. Found: " << b << " cookies";
  LOG(WARNING) << "Hello glog. Found: " << b << " cookies";
  LOG(ERROR) << "Hello glog. Found: " << b << " cookies";

  ROS_INFO("Hello Ros");

  LOG(FATAL) << "Hello glog. Found: " << b << " cookies";




}
