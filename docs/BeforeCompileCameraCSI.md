sudo apt install libglew-dev
gh repo clone dusty-nv/jetson-utils
cd jetson-utils && mkdir build && cd build
cmake ..
make -j6
sudo make install
