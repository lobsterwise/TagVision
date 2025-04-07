use std::collections::HashMap;

use image::{GrayImage, Pixel, Rgb, RgbImage};

/// Maintains reusable allocations for multiple image sizes
pub struct ImageAllocator {
	gray_images: HashMap<(u32, u32), GrayImage>,
	rgb_images: HashMap<(u32, u32), RgbImage>,
}

impl ImageAllocator {
	pub fn new() -> Self {
		Self {
			gray_images: HashMap::new(),
			rgb_images: HashMap::new(),
		}
	}

	pub fn get_gray_image(&mut self, width: u32, height: u32) -> &mut GrayImage {
		self.gray_images
			.entry((width, height))
			.or_insert(GrayImage::new(width, height))
	}

	pub fn get_rgb_image(&mut self, width: u32, height: u32) -> &mut RgbImage {
		self.rgb_images
			.entry((width, height))
			.or_insert(RgbImage::new(width, height))
	}
}

/// A faster version of converting a gray image to an RGB one that can reuse an existing allocation
pub fn fast_gray_to_rgb(input: &GrayImage, output: &mut RgbImage) {
	debug_assert_eq!(input.width(), output.width());
	debug_assert_eq!(input.height(), output.height());

	for x in 0..input.width() {
		for y in 0..input.height() {
			let luma = input[(x, y)].0[0];
			let starting_index = (y as usize * input.width() as usize + x as usize) * 3;
			// Assign to all of the rgb values
			unsafe {
				let starting_ptr = output.as_mut_ptr().add(starting_index);
				*starting_ptr = luma;
				*starting_ptr.add(1) = luma;
				*starting_ptr.add(2) = luma;
			}
		}
	}
}
