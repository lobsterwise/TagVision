# Builds the Apriltag library and TagVision binary for the Orange Pi.
# You must have aarch64-linux-gnu-gcc and aarch64-linux-gnu-g++ installed with sysroot at /usr/aarch64-linux-gnu
#
# NOTE: You must remove the apriltag/build directory if you are changing from compiling
# native to compiling cross for the Orange Pi, otherwise it will reuse the old configuration

cd apriltag
cmake -B build \
	-DCMAKE_TOOLCHAIN_FILE=../Toolchain-OrangePi.cmake \
	-DCMAKE_INSTALL_PREFIX=/usr \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DBUILD_SHARED_LIBS=OFF \
	-DBUILD_EXAMPLES=OFF \
	-DBUILD_PYTHON_WRAPPER=OFF
sudo DESTDIR=/usr/aarch64-linux-gnu cmake --build build --target install
cd ..

cd IPPE/cpp
cmake -B build \
	-DCMAKE_TOOLCHAIN_FILE=../../Toolchain-OrangePi.cmake \
	-DCMAKE_INSTALL_PREFIX=/usr \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DBUILD_SHARED_LIBS=OFF \
	-DBUILD_EXAMPLES=OFF
sudo DESTDIR=/usr/aarch64-linux-gnu cmake --build build --target install
cd ../..

cargo build --profile deploy --target aarch64-unknown-linux-gnu --no-default-features --features native
