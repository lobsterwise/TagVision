use nalgebra::{Matrix3, Matrix3x4};

use super::geom::PnPSolution;

pub mod p3p;
pub mod polynomial;

pub trait PnPSolver {
	fn from_matrix(mat: Matrix3<f64>) -> Self
	where
		Self: Sized,
	{
		Self::new(mat[(0, 0)], mat[(1, 1)], mat[(0, 2)], mat[(1, 2)])
	}

	fn new(fx: f64, fy: f64, cx: f64, cy: f64) -> Self
	where
		Self: Sized;

	fn solve(&mut self, object_points: Matrix3x4<f64>, rays: Matrix3x4<f64>)
		-> Option<PnPSolution>;
}
