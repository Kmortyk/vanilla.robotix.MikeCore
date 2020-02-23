## From:

https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674

## Issue:

Traceback (most recent call last):
  File "/catkin_ws/src/ros_python3_issues/src/issue_cv_bridge.py", line 23, in <module>
    ros_image = bridge.cv2_to_imgmsg(numpy.asarray(empty_image), encoding="rgb8") # convert PIL image to ROS image
  File "/opt/ros/melodic/lib/python2.7/dist-packages/cv_bridge/core.py", line 259, in cv2_to_imgmsg
    if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
  File "/opt/ros/melodic/lib/python2.7/dist-packages/cv_bridge/core.py", line 91, in encoding_to_cvtype2
    from cv_bridge.boost.cv_bridge_boost import getCvType
ImportError: dynamic module does not define module export function (PyInit_cv_bridge_boost)

## Build cv_bridge:

sudo apt-get install python-catkin-tools python3-dev python3-numpy

mkdir ~/catkin_build_ws && cd ~/catkin_build_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
catkin config --install

mkdir src
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git

cd ~/catkin_build_ws
catkin build cv_bridge
source install/setup.bash --extend

## Fix:

I was able to successfully compile cv_bridge with opencv4 below are the rough notes of what i did:

1) Add set (CMAKE_CXX_STANDARD 11) to your top level cmake
2) In cv_bridge/src CMakeLists.txt line 35 change to if (OpenCV_VERSION_MAJOR VERSION_EQUAL 4)
3) In cv_bridge/src/module_opencv3.cpp change signature of two functions    

a) UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, int flags, UMatUsageFlags usageFlags) const
   to
   UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, AccessFlag flags, UMatUsageFlags usageFlags) const
    
b) bool allocate(UMatData* u, int accessFlags, UMatUsageFlags usageFlags) const
   to
   bool allocate(UMatData* u, AccessFlag accessFlags, UMatUsageFlags usageFlags) const
