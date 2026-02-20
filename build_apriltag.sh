#!/bin/sh
cd apriltag
	# -DCMAKE_TOOLCHAIN_FILE=../Toolchain-Windows.cmake \
cmake -B build \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DBUILD_SHARED_LIBS=OFF \
	-DBUILD_EXAMPLES=OFF \
	-DBUILD_PYTHON_WRAPPER=OFF \
	-DUSE_SSE=ON
sudo cmake --build build --target install

# cd ../sqpnp
# cmake -B build \
# 	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
# 	-DBUILD_SHARED_LIBS=OFF \
# 	-DBUILD_EXAMPLES=OFF \
# 	-DBUILD_PYTHON_WRAPPER=OFF \
# 	-DUSE_SSE=ON
# sudo cmake --build build --target install
