use std::{ffi::*, fmt::Debug};

#[link(name = "apriltag", kind = "static", modifiers = "+whole-archive")]
extern "C" {
	pub fn apriltag_detector_create() -> *mut _AprilTagDetector;
	pub fn apriltag_detector_add_family_bits(
		td: *mut _AprilTagDetector,
		fam: *mut _AprilTagFamily,
		bits_corrected: c_int,
	);
	pub fn apriltag_detector_destroy(td: *mut _AprilTagDetector);
	pub fn apriltag_detector_detect(
		td: *mut _AprilTagDetector,
		img_orig: *mut _ImageU8,
	) -> *mut _ZArray;
	pub fn apriltag_detections_destroy(detections: *mut _ZArray);
	pub fn tag36h11_create() -> *mut _AprilTagFamily;
	pub fn tag36h11_destroy(tf: *mut _AprilTagFamily);
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct _AprilTagFamily {
	pub ncodes: u32,
	pub codes: *const u64,
	pub width_at_border: c_int,
	pub total_width: c_int,
	pub reversed_border: bool,
	pub nbits: u32,
	pub bit_x: *const u32,
	pub bit_y: *const u32,
	pub h: u32,
	pub name: *const c_char,
	pub r#impl: *const c_void,
}

#[derive(Debug)]
#[repr(C)]
pub struct _AprilTagQuadThreshParams {
	pub min_cluster_pixels: c_int,
	pub max_nmaxima: c_int,
	pub critical_rad: c_float,
	pub cos_critical_rad: c_float,
	pub max_line_fit_mse: c_float,
	pub min_white_black_diff: c_int,
	pub deglitch: c_int,
}

#[derive(Debug)]
#[repr(C)]
pub struct _AprilTagDetector {
	pub nthreads: c_int,
	pub quad_decimate: c_float,
	pub quad_sigma: c_float,
	pub refine_edges: c_int,
	pub decode_sharpening: c_double,
	pub debug: bool,
	pub qtp: _AprilTagQuadThreshParams,
	pub tp: *mut _TimeProfile,
	pub nedges: u32,
	pub nsegments: u32,
	pub nquads: u32,
	pub tag_families: *mut _ZArray,
	pub wp: *mut _WorkerPool,
	pub mutex: DebuggablePthreadMutex,
}

#[repr(C)]
pub struct _AprilTagDetection {
	pub family: *mut _AprilTagFamily,
	pub id: c_int,
	pub hamming: c_int,
	pub decision_margin: c_float,
	pub h: *mut _MatD,
	/// Center in pixel coordinates
	pub c: [c_double; 2],
	/// Corners in pixel coordinates, counter-clockwise
	pub p: [[c_double; 2]; 4],
}

#[repr(C)]
pub struct _MatD {
	pub nrows: c_uint,
	pub ncols: c_uint,
	pub data: *mut c_double,
}

#[repr(C)]
pub struct _TimeProfileEntry {
	pub name: [c_char; 32],
	pub utime: i64,
}

impl _TimeProfileEntry {
	pub fn name(&self) -> &str {
		unsafe {
			let name = std::slice::from_raw_parts(self.name.as_ptr() as *const u8, 32);
			std::str::from_utf8_unchecked(name)
		}
	}
}

impl Debug for _TimeProfileEntry {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		let name = self.name();
		write!(f, "({name}; @{})", self.utime)
	}
}

#[repr(C)]
pub struct _TimeProfile {
	utime: i64,
	data: *const _ZArray,
}

impl _TimeProfile {
	pub fn report(&self) {
		let mut last_time = None;
		unsafe {
			for entry in (*self.data).iter::<_TimeProfileEntry>() {
				if let Some(last_time) = last_time {
					let delta = entry.utime - last_time;
					let time = if delta >= 1000 {
						format!("{:.2}ms", delta as f32 / 1000.0)
					} else {
						format!("{delta}us")
					};
					println!("{} - {time}", entry.name());
				}
				last_time = Some(entry.utime);
			}
		}
	}
}

impl Debug for _TimeProfile {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		unsafe {
			for entry in (*self.data).iter::<_TimeProfileEntry>() {
				writeln!(f, "{entry:?},")?;
			}
		}

		Ok(())
	}
}

#[derive(Debug)]
#[repr(C)]
pub struct _WorkerPool {
	pub nthreads: c_int,
	pub tasks: *mut _ZArray,
	pub taskspos: c_int,
	pub threads: *mut libc::pthread_t,
	pub status: *mut c_int,
	pub mutex: DebuggablePthreadMutex,
	pub startcond: DebuggablePthreadCond,
	pub start_predicate: bool,
	pub endcond: DebuggablePthreadCond,
	pub end_count: c_int,
}

#[repr(C)]
pub struct _ImageU8 {
	pub width: i32,
	pub height: i32,
	pub stride: i32,
	pub buf: *mut u8,
}

#[repr(C)]
pub struct _ZArray {
	pub el_sz: libc::size_t,
	pub size: c_int,
	pub alloc: c_int,
	pub data: *mut c_char,
}

impl _ZArray {
	pub unsafe fn get<'a, T>(&'a self, index: usize) -> Option<&'a T> {
		let size = self.size as usize;

		if index >= size {
			return None;
		}

		let data = self.data as *mut T;
		let data = std::slice::from_raw_parts(data, size);

		Some(&data[index])
	}

	pub unsafe fn iter<'a, T: 'a>(&'a self) -> impl Iterator<Item = &'a T> + 'a {
		(0..self.size as usize).map(|x| unsafe { self.get(x).unwrap_unchecked() })
	}
}

impl Debug for _ZArray {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(
			f,
			"Size: {}, Elem Size: {}, Alloc: {}, Data: {:?}",
			self.size, self.el_sz, self.alloc, self.data
		)
	}
}

#[repr(transparent)]
pub struct DebuggablePthreadMutex(libc::pthread_mutex_t);
impl Debug for DebuggablePthreadMutex {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(f, "")
	}
}

#[repr(transparent)]
pub struct DebuggablePthreadCond(libc::pthread_cond_t);
impl Debug for DebuggablePthreadCond {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(f, "")
	}
}
