# Setup notes

1. Install JetPack per the instructions on [Nvidia's developer site](http://docs.nvidia.com/jetpack-l4t/index.html#developertools/mobile/jetpack/l4t/3.2rc/jetpack_l4t_install.htm)
2. Packages installed -- pip, pip3, numpy (and numpy for py 3), ccache, nano, eric (IDE) (all with apt-get)
3. Install GNU7 (updated C++ compiler) with ```
sudo apt-get remove gcc-5 gcc++-5
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y gcc-7 g++-7
cd /usr/bin
sudo ln -s aarch64-linux-gnu-gcc-7 gcc
```
4. Install build-essentials: ```
sudo apt-get install build-essential
```
5. Install OpenCV per the instructions on [this blog](https://jkjung-avt.github.io/opencv3-on-tx2/) (here's the make command minus the line breaks) ```
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_CUDA=ON -D CUDA_ARCH_BIN="6.2" -D CUDA_ARCH_PTX="" -D WITH_CUBLAS=ON -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON -D ENABLE_NEON=ON -D WITH_LIBV4L=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF ..
```
6. Install gstreamer: ``` sudo apt-get install gstreamer1.0-plugins-bad ```

Other notes:

* Test the on-board camera with ```
gst-launch-1.0 nvcamerasrc ! 'video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1, format=NV12' ! nvvidconv flip-method=2 ! nvegltransform ! nveglglessink
```
