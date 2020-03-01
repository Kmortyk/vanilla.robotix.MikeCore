nano sources/jetson-utils/camera/gstCamera.cpp
cd sources/jetson-utils/
rm -rf build
mkdir build
cd build
cmake ..
make -j4
sudo make install

gst-launch-1.0 nvarguscamerasrc wbmode=9 ! 'video/x-raw(memory:NVMM),width=3820, height=2464, framerate=21/1, format=NV12' ! nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=616' ! nvvidconv ! nvegltransform ! nveglglessink -e
