CATKIN_PATH="$(which catkin)"

if [ "$CATKIN_PATH" = "" ]; then
    echo "Installing ros noetic"

    git clone https://github.com/lucasw/ros_from_src.git
    mkdir build
    cd build
    ROSCONSOLE=https://github.com/ros/rosconsole ../ros_from_src/git_clone.sh
    sudo ../ros_from_src/dependencies.sh
    ../ros_from_src/build.sh

else
  sudo apt install -y libgoogle-glog-dev
  echo "catkin found"
fi

export DEBIAN_FRONTEND=noninteractive
BUILD_DIR=cmake-build

if test -d "$BUILD_DIR"; then
  rm -r $BUILD_DIR
else
  mkdir $BUILD_DIR
fi


cmake CMakeLists.txt -B $BUILD_DIR/
cd $BUILD_DIR || exit
cmake --build . --target test_default -- -j 6
cmake --build . --target test_glog -- -j 6
cmake --build . --target test_lpp -- -j 6
cmake --build . --target test_lpp_custom -- -j 6
cmake --build . --target test_roslog -- -j 6
cd devel/lib/lpp
./test_default
./test_roslog
./test_lpp
./test_lpp_custom
./test_roslog