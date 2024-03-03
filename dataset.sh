#!/bin/bash

cd
cd orb_to_depth/
source devel/setup.bash
rosrun ofslam mono_tum src/ofslam/Vocabulary/ORBvoc.bin src/ofslam/Examples/Monocular/ip640_undistoried.yaml /home/zjd/shujuji/ip24/

cd
cd offline_reconstruction/newmvs_build
./bin/DensifyPointCloud -w /home/zjd/shujuji/ip24of+mvs/reconstruction/ -i sfm.txt -o dense.mvs
