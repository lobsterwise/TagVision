use std::process::Command;

fn main() {
	// Command::new("./build_apriltag.sh")
	// 	.spawn()
	// 	.unwrap()
	// 	.wait()
	// 	.unwrap();
	println!("cargo:rustc-link-search=./apriltag/build/");
	// println!("cargo:rustc-link-lib=static:+whole-archive=apriltag");
}
