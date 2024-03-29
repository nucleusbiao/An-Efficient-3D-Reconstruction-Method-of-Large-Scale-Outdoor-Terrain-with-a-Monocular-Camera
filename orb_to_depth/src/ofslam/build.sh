echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Configuring and building orb_to_depth ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 

cd ..

echo "Converting vocabulary to binary"
./tools/bin_vocabulary
