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

  int occasion = 5;

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

  std::cout << "----------- Occasional logging -----------" << std::endl;
  usleep(1e5);
  for (int i = 0; i < occasion; i++) {
    LOG_EVERY(I, occasion, "Occasional log");
    LOG_EVERY(W, occasion, "Occasional log");
    LOG_EVERY(E, occasion, "Occasional log");
    LOG_EVERY_N(INFO, occasion) << "Occasional log";
    LOG_EVERY_N(WARNING, occasion) << "Occasional log";
    LOG_EVERY_N(ERROR, occasion) << "Occasional log";

    //TODO Implement
    /*
    ROS_INFO_THROTTLE(occasion, "Hello Ros");
    ROS_INFO_STREAM_THROTTLE(occasion, "" << "Occasional log");
    ROS_WARN_THROTTLE(occasion, "Hello Ros");
    ROS_WARN_STREAM_THROTTLE(occasion, "" << "Occasional log");
    ROS_ERROR_THROTTLE(occasion, "Hello Ros");
    ROS_ERROR_STREAM_THROTTLE(occasion, "" << "Occasional log");
    */
  }

  usleep(1e6);
  std::cout << "----------- First N occurrences -----------" << std::endl;
  occasion = 5;
  int first_n_occurrences = 2;
  for (int i = 0; i < occasion * 2; i++) {


    LOG_FIRST(I, first_n_occurrences, "Log++ First " << first_n_occurrences << " occurrences");
    LOG_EVERY(I, occasion, "Log every " << occasion << " occasion");
    LOG_FIRST(W, first_n_occurrences, "Log++ First " << first_n_occurrences << " occurrences");
    LOG_FIRST(E, first_n_occurrences, "Log++ First " << first_n_occurrences << " occurrences");


    LOG_FIRST_N(INFO, first_n_occurrences) << "Log INFO First " << first_n_occurrences << " occurrences";
    LOG_FIRST_N(WARNING, first_n_occurrences) << "Log WARNING First " << first_n_occurrences << " occurrences";
    LOG_FIRST_N(ERROR, first_n_occurrences) << "Log ERROR First " << first_n_occurrences << " occurrences";
  }
}
