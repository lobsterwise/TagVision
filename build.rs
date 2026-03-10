fn main() {
	println!("cargo:rustc-link-search=./apriltag/build/");
	println!("cargo:rustc-link-search=./IPPE/cpp/build/lib/");
	// println!("cargo:rustc-env=LIBCLANG_PATH=/usr/lib/libclang.so");

	println!("cargo:include=/usr/include/opencv4/");
	println!("cargo:rustc-link-search=native=/usr/lib");

	println!("cargo:rustc-link-lib=dylib=stdc++");
	println!("cargo:rustc-link-lib=dylib=opencv_core");
	println!("cargo:rustc-link-lib=dylib=opencv_calib3d");
	println!("cargo:rustc-link-lib=dylib=opencv_imgproc");
}
