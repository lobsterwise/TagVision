# TagVision

TagVision is a high-performance and high-reliability pipeline for estimating robot pose from AprilTags on FRC fields. It is designed to never fail and be easy to set up, while also maximizing the amount of bandwidth you can get out of your budget. All you need is an Orange Pi and a few cheap cameras.

## Features
- Can handle high-speed data from multiple cameras at once, each running on different threads to prevent deadlocks or propagating failures
- Handles UVC or V4L2 cameras with deterministic USB port indexing to ensure reliable device identification. Also configures cameras on startup with exposure and brightness parameters to ensure consistent frames.
- Uses Infinite Planar Pose Estimation and 9-parameter camera undistortion to calculate a stable pose estimate from each tag and averages them together with covariance statistics to use in k-filter estimations
- Runs official libAprilTag detection distributed between multiple worker tasks
- Connects to a robot-hosted NetworkTables 4 server and sends timestamped pose and tag updates
- Catches and logs network and capture errors, while eventually restarting flaky modules or NT connections to clear sticky errors and improve uptime
- Supports local testing on Linux and Windows and deployment to an aarch processor like an Orange Pi or Raspberry Pi
- Automatic setup of systemd scripts to run on startup
- Serves a live mjpeg/http camera feed with drawn detections to aid debugging or driver visibility
- Supports multiple field layouts and filtering parameters to reject bad estimations

## Setup
### Hardware

The recommended setup is:
- [Orange Pi 5b](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-5B.html)
	- Probably safe with 8GB of RAM but you can also do 16GB or maybe even 4GB
	- Any other coprocessor with a good CPU will work
	- WiFi is optional but will make setup easier
	- You can find lots of cheap resellers on Amazon
- 1-3 [ArduCam global shutter grayscale 50FPS 2MP cameras](https://www.arducam.com/arducam-2mp-global-shutter-usb-camera-board-for-computer-50fps-ov2311-monochrome-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-compatible-with-windows-linux-android-and-mac-os.html)
	- Any other FoV, resolution, FPS will work but grayscale is cheaper (and all that is necessary), and global shutter is **required**
- 5V Voltage Regulator
	- We used [these](https://www.pololu.com/product/2851) from Pololu, but anything else will work. Just check the input/output voltage and current specs.
- USB-C pigtail cable
	- Used to connect the voltage regulator to the power input of the Orange Pi
- Orange Pi case
	- Metal ones are nice but you can also 3D print. Just make sure it can fit a fan.
- Heat sinks
	- You can buy a pack of tiny heat sinks with sticky tape. Put them on the CPU to help cooling.
- Case fan
	- This is absolutely required. Without it, the Pi will thermal throttle and crash within minutes of running the very intensive vision pipeline.
	- Solder it to your power and ground pins on your Pi, the ribbon cable ones that connect to the fan port don't seem as reliable to me, and you'll be running it at max power all the time anyway.
- Other cables
	- You need wire to connect to the regulator to a Mini PDP and an Ethernet cable to connect to your radio

How much this costs is mostly dependent on how many cameras you want. More cameras = more visibility and angles at the cost of some FPS and money. Orange Pi used to be a lot cheaper of course, now it will be ~$200. Each camera will be ~$100.

### Getting the Orange Pi Ready

This is the most intensive part. You will probably encounter some additional issues along the way with downloading / installing, just work your way through it.

1. Powering the Pi
You can do this with a bench power supply but since you are already going to be using it later, you might as well set up your regulator in your robot's power network and connect the Pi there. Make sure you connect the regulator to a mini PDP.
2. Setting up the OS
There are lots of options for distributions you can install on your Orange Pi. I think the most reliable one is the official Orange Pi OS which can be downloaded [here](https://drive.usercontent.google.com/download?id=10LL5_MKOe0is_TnDWkIsIJFrgbQDgr05&export=download&authuser=0&confirm=t&uuid=b06ffb9d-2aa7-4ed0-8d03-e308cdc2f357&at=AGN2oQ2fwIjxU9mY9zdCklupIqj6:1773186459141).

Flash the image onto a blank MicroSD card using Balena Etcher and boot the Pi with it inserted.
3. Connecting to the Pi
This is easiest if you just use the robot radio ethernet, but it can be done over a router. The tough part is finding the Orange Pi's IP address. A lot of the time you can connect using `orangepi5b.local` or `orangepi.local`, but you can also use Advanced IP Scanner to find it. If it is on your robot radio you can set the IP scan range to `10.XX.XX.1-10.XX.XX.255` (X's being your team number).

Once you know the IP you can SSH in with username `orangepi` and password `orangepi` (only if you use the official OS image of course).
4. Connecting to the internet
I hope to statically link OpenCV into the binary so no download is needed in the future, but for now we need to install it. This is easiest if you bought a model with WiFi built in, but you can also connect to your router over ethernet or use USB tethering on an Android phone. Use `nmcli` to connect to the internet (there are guides online).

Importantly, once you connect to the internet, you need to either modify the network routing priority or disconnect the radio connection, otherwise the device will try to connect to the internet through the radio (impossible).
5. Installing OpenCV
The only problem with the official Orange Pi OS image is that it uses broken Huawei repositories for packages.

Do `sudo vim` or `sudo nano` to edit `/etc/apt/sources.list`. Comment out all of the lines in the file with `#`'s. Replace the contents of the file with:
```
deb http://archive.ubuntu.com/ubuntu/ jammy main restricted universe multiverse  
deb http://archive.ubuntu.com/ubuntu/ jammy-updates main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu/ jammy-backports main restricted universe multiverse
deb http://security.ubuntu.com/ubuntu jammy-security main restricted universe multiverse
```

Then run `sudo apt update` and `sudo apt upgrade`. Finally, you can install OpenCV with `sudo apt install libopencv-dev`.
### Installing TagVision
1. Clone the repo and prepare
Clone the repo locally with `git clone https://github.com/lobsterwise/TagVision.git`. This will be much easier on Linux or WSL since it makes it easier to install the cross-platform GCC and run the install scripts.

You will need the [Rust toolchain](https://rustup.rs/) with the `aarch64-unknown-linux-gnu` toolchain, CMake, and OpenCV (libopencv). Then you need to install the aarch64 GCC to cross-compile with. For example: `sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu`.
2. Build
Run `./build_orangepi.sh` in the repository. If it asks you about any other missing libraries install those as well.
3. Deploy
Now that the executable is built you need to copy it to the Orange Pi using `scp`. For example: `scp target/aarch64-unknown-linux-gnu/deploy/tag_vision orangepi@10.41.45.122:/home/orangepi/tag_vision`.
4. Test
Run `./tag_vision` wherever you put the file on your Pi. Hopefully, you should just see an error about a missing config file, rather than linkage errors.
### Configuring
1. Create the file
Create a config file in somewhere it will be safe (your home dir makes sense) with the name `config.json`. Start with this:
```json
{
	"network": {
		"name": "TagVision",
		"address": "10.41.45.2",
		"enable_wpi_schemas": false,
		"camera_server": true
	},
	"detector_params": {
		"quad_decimate": 3.0,
		"thread_count": 8
	},
	"tags": {
		"layout": "2026_welded",
		"tag_size": 6
	},
	"modules": {}
}
```

Replace the `address` with your actual RoboRIO address, which will be `10.TE.AM.2`. `network.name` is what the base NetworkTable will be called, which you can change to whatever you want.
2. Add a module
Modules are simply cameras which are in your setup. Add one with an ID used to identify it to the `modules` map like this:
```json
{
	"modules": {
		"left_cam": {
			"camera": {
				"device_id": "platform-fc880000.usb-usb-0:1:1.0",
				"backend": "native",
				"width": 1600,
				"height": 1200,
				"fps": 50,
				"intrinsics": {}
			}
		}
	}
}
```

The device ID is used to find the exact camera device and is based on USB port. Find available devices by doing `ls /dev/v4l/by-path`and identify them by plugging/unplugging them. The ID will be whatever one of those filenames are without the `-video-index0/1` part at the end. Since cameras are identified by USB port, you should probably mark the camera cable and port with marker or tape to ensure you don't mix them up!

Set the width, height, and FPS to the correct values.
3. Add calibration
Calibrate your camera using a laptop with a camera, a simple camera app with video recording, and the official WPICal tool. Whatever app you use to record the video, make sure you have the correct FPS and resolution configuration or the calibration will be useless. Also make sure your camera lens is in focus before you begin and that you don't move the calibration board too quickly or you will have wasted your time.

Once WPICal generates the intrinsics JSON in the same folder your video was in, you can simply paste it in your configuration file like this:
```json
"camera": {
	"intrinsics": {
		"camera_matrix": [
			909.0258045807326,
			0.0,
			801.7450730805796
			0.0,
			909.1320924435286,
			693.451312745887
			0.0,
			0.0,
			1.0
		],
		"distortion_coefficients": [
			0.028258634497674625,
			-0.02930599098921872,
			0.00031602445745346517,
			-0.0006249646695728876,
			0.06850817716872978,
			...
		]
	}
}
```
4. Fine Tuning
You can tweak `quad_decimate` and `thread_count` a bit to find whats the fastest for your system. Quad decimation will reduce image quality and detection accuracy quite a bit but will also improve performance quite a bit. 1-3 will be minimal quality loss for a great performance gain, but beyond that can make your poses less accurate. Thread count can be anywhere from 1-16, just see what works for you across a long running time.

### Handling events on your robot
I would recommend using WPILib's pose estimator or a timestamped k-filter estimator if you are feeling more advanced. This system works best when integrating with good odometry.

There is a library file you should copy into your code in the root of the repo called `TagVision.java`. Use WPILib StructArraySubscribers and StructSubscribers to poll the data and add it to your vision system. You might have to set `enable_wpi_schemas` to `true` in your config.

### Setting up the service
To automatically start the vision server on boot, cd to where the executable is on your Pi and run `./tag_vision setup <config_path>`, where `config_path` is where you put your configuration file.
