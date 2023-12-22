//
// Created by 4c3y (acey) on 18.08.22.
//

#include <log++.h>

int main([[maybe_unused]] int argc, char **argv) {
  LOG_INIT(argv[0]);

  int foo = 5;

  LOG(I, "Foo: " << foo);           //Log++ syntax
  ROS_INFO_STREAM("Foo: " << foo);  //ROS syntax
  LOG(INFO) << "Foo: " << foo;      //Glog syntax
  return 0;
}