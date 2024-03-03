#!/bin/bash

cd
cd orb_to_depth/
source devel/setup.bash
rosrun ofslam myvideo

cd
cd offline_reconstruction/newmvs_build
./bin/DensifyPointCloud -w /home/zjd/shujuji/ip24of+mvs/reconstruction/ -i sfm.txt -o dense.mvs
