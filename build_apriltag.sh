#!/bin/sh
cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SHARED_LIBS=OFF
sudo cmake --build build --target install