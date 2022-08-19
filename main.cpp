#include <iostream>

#include "ros/ros.h"
#include "log++.h"
#include "glog/logging.h"

using namespace lpp;

int main(int argc, char **argv) {

  LOG_INIT(argv[0], Mode::DEFAULT)
  FLAGS_logtostderr = true;


  int a = 3;
  int b = 2;
  LOG(I, "Hello log++. Found: " << a << " cookies")
  LOG(INFO) << "Hello glog. Found: " << b << " cookies";
  ROS_INFO("Hello Ros");
  return 0;
}
