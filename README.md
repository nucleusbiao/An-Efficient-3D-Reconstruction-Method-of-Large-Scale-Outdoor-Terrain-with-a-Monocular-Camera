# An-Efficient-3D-Reconstruction-Method-of-Large-Scale-Outdoor-Terrain-with-a-Monocular-Camera
This repository is a framework for large-scale outdoor scene reconstruction using a monocular camera

The code includes two parts: real-time reconstruction and fast offline and accurate reconstruction of farmland scenes. 

The real-time reconstruction module is placed in the orb_to_depth directory and is a ROS workspace, which includes the visual odometry module of SLAM function package and the monocular depth estimation module of Zoetodepth function package.
The fast offline reconstruction module is placed in the offline_reconstruction folder, and both modules need to be compiled
## Building

```
cd orb_to_depth/src/ofslam
./build.sh
cd ../..
catkin_make
cd ../offline_reconstruction/newmvs_build
cmake . ../ -DCMAKE_BUILD_TYPE=Release
sudo make -j4 && sudo make install
```

## Run

### Real time reconstruction module

````
roscore

cd orb_to_depth
source devel/setup.bash
rosrun zoetodepth server.py

rosrun ofslam mono_tum src/ofslam/Vocabulary/ORBvoc.bin src/ofslam/Examples/Monocular/ip640_undistoried.yaml /home/zjd/shujuji/ip24/(Dataset format)
rosrun ofslam myvideo(Video streaming format)
rosrun ofslam myusb(Real time operation mode of the camera)
````

### Fast offline precise reconstruction module

The real-time reconstruction module will save keyframes and sfm.txt files after running, and place them in the same directory as input for fast and accurate offline reconstruction.

````
./bin/DensifyPointCloud -w /home/zjd/shujuji/ip24of+mvs/reconstruction/ -i sfm.txt -o dense.mvs
````

### script files

For ease of operation, we save  the running process of the dataset format and video stream format to script files.

````
./dataset.sh
./video.sh
````
