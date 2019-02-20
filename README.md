Simulation of Kinect Measurements
=============

This C++ project implements the Kinect sensor model as described in 

**BlenSor: Blender Sensor Simulation Toolbox.** *Gschwandtner, Michael and Kwitt, Roland and Uhl, Andreas and Pree, Wolfgang. In Advances in Visual Computing. Lecture Notes in Computer Science. pp 199--208. 2011.*

and as implemented in python as a Blender plugin ([BlenSor Webpage](http://www.blensor.org)).

The simulated measurement shows typical artifacts of a kinect, e.g., occlusion boundaries due to distance between IR projector and IR camera, 8 bit quantisation, smooting within a 9x9 pixel correlation window. This implementation also includes options for adding either Gaussian, Perlin or Simplex noise.

This implementation is simplified in that it accepts only a single rigid object as input.

If you are using this code, please cite our work:

**Robot Arm Pose Estimation through Pixel-Wise Part Classification.** *Jeannette Bohg, Javier Romero, Alexander Herzog and Stefan Schaal. Proceedings of the 2014 IEEE International Conference on Robotics and Automation. pp 3143--3150. 2014.*

Code borrowed from: https://github.com/rogersce/cnpy

Requirements
----------
The following libraries are required to compile the code:

* OpenCV (image I/O, filtering)
* CGAL (Fast Intersection Queries)
* Eigen (Linear Algebra)
* assimp (Mesh I/O)
* noise (Generation of Different Noise Types)

The following libraries are optional:

* OpenMP (Parallelization)
* PCL (Point cloud  I/O)

Setup
------------
Install OpenCV 3 on Ubuntu 16.04. Adapted from [here](https://www.learnopencv.com/install-opencv3-on-ubuntu/)

##### Step 1: Install OS libraries

```bash
sudo apt-get install build-essential checkinstall cmake pkg-config yasm
sudo apt-get install git gfortran
sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
 
# If you are using Ubuntu 14.04
sudo apt-get install libtiff4-dev
# If you are using Ubuntu 16.04
sudo apt-get install libtiff5-dev
 
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install libxine2-dev libv4l-dev
sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install qt5-default libgtk2.0-dev libtbb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install libvorbis-dev libxvidcore-dev
sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install x264 v4l-utils
 
# Optional dependencies
sudo apt-get install libprotobuf-dev protobuf-compiler
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```

##### Step 2: Compile and Install OpenCV3

```bash
mkdir -p $HOME/dependencies
cd $HOME/dependencies
git clone https://github.com/opencv/opencv.git -b 3.3.1
git clone https://github.com/opencv/opencv_contrib.git -b 3.3.1
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
make -j$(nproc)
sudo make install -j$(nproc)
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
``` 

##### Step 3: Create symlink in virtual environment

```bash
$ find /usr/local/lib/ -type f -name "cv2*.so"
############ For Python 2 ############
## binary installed in dist-packages
/usr/local/lib/python2.6/dist-packages/cv2.so
/usr/local/lib/python2.7/dist-packages/cv2.so
## binary installed in site-packages
/usr/local/lib/python2.6/site-packages/cv2.so
/usr/local/lib/python2.7/site-packages/cv2.so
  
############ For Python 3 ############
## binary installed in dist-packages
/usr/local/lib/python3.5/dist-packages/cv2.cpython-35m-x86_64-linux-gnu.so
/usr/local/lib/python3.6/dist-packages/cv2.cpython-36m-x86_64-linux-gnu.so
## binary installed in site-packages
/usr/local/lib/python3.5/site-packages/cv2.cpython-35m-x86_64-linux-gnu.so
/usr/local/lib/python3.6/site-packages/cv2.cpython-36m-x86_64-linux-gnu.so
```

Double check the directory before running these commands:

```bash
############ For Python 2 ############
cd $HOME/.local/lib/python2.7/site-packages
ln -s /usr/local/lib/python2.7/dist-packages/cv2.so cv2.so
  
############ For Python 3 ############
cd $HOME/.local/lib/python3.6/site-packages
ln -s /usr/local/lib/python3.6/dist-packages/cv2.cpython-36m-x86_64-linux-gnu.so cv2.so
```

Compilation
------------

The compilation of this code is tested on Ubuntu (12.04 precise, 12.10 quantal, and 16.04 xenial). The CMakeLists.txt file might need to be slightly adapted for correctly linking against PCL dependent on whether you installed it as a standalone library or with ROS.


```bash
mkdir build
cd build
cmake ..
make -j
```

Testing
------------
There is a small test program that will render the simulated kinect measurements of a wheel at a number of slightly perturbed transformations relative to the camera.

```bash
cd bin
./render_object wheel.obj
```

This should store a number of depth and labeled images in /tmp. If you have PCL installed, it also stores point clouds as pcl files.

Point clouds generated from a simulated kinect measurement taken from a wheel in 10 slightly different poses. This measurement does not expose additional noise.

![](data/Wheels.png?raw=true)
