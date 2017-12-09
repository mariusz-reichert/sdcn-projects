#!/bin/bash

if [ ! -d ./build ]; then
  mkdir build
fi

cd build
cmake .. && make

./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt output.txt
cd ..