export DEBIAN_FRONTEND=noninteractive
sudo apt install -y libgoogle-glog-dev 
BUILD_DIR=cmake-build
mkdir $BUILD_DIR
cmake CMakeLists.txt -B $BUILD_DIR/
cd $BUILD_DIR || exit
cmake --build . --target test_default -- -j 6
cmake --build . --target test_glog -- -j 6
cmake --build . --target test_lpp -- -j 6
cmake --build . --target test_roslog -- -j 6
./test_default
./test_roslog
./test_lpp
./test_roslog