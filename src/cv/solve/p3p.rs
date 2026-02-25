#![allow(non_snake_case)]

use nalgebra::{Matrix3, Matrix3x1, Matrix3x4, Matrix4, Matrix4x1};

use crate::cv::{
	apriltag::{layout::AprilTagLayout, AprilTagDetection},
	distort::OpenCVCameraIntrinsics,
	geom::{PnPSolution, Pose3D, Pose3DWithError},
};

use super::{polynomial::solve_deg4, PnPSolver};

pub struct P3P;

impl PnPSolver for P3P {
	fn new() -> Self {
		Self
	}

	fn solve(
		&mut self,
		layout: &AprilTagLayout,
		detection: &AprilTagDetection,
		intrinsics: &OpenCVCameraIntrinsics,
		tag_width: f64,
	) -> Option<PnPSolution> {
		let object_points = layout.get_tag_corners(detection.id, tag_width)?;
		let rays = detection.get_undistorted_corners_rays(intrinsics);

		let rx0 = rays[(0, 0)];
		let ry0 = rays[(1, 0)];
		let rz0 = rays[(2, 0)];
		let rx1 = rays[(0, 1)];
		let ry1 = rays[(1, 1)];
		let rz1 = rays[(2, 1)];
		let rx2 = rays[(0, 2)];
		let ry2 = rays[(1, 2)];
		let rz2 = rays[(2, 2)];
		let rx3 = rays[(0, 3)];
		let ry3 = rays[(1, 3)];
		let rz3 = rays[(2, 3)];

		let X0 = object_points[(0, 0)];
		let Y0 = object_points[(1, 0)];
		let Z0 = object_points[(2, 0)];
		let X1 = object_points[(0, 1)];
		let Y1 = object_points[(1, 1)];
		let Z1 = object_points[(2, 1)];
		let X2 = object_points[(0, 2)];
		let Y2 = object_points[(1, 2)];
		let Z2 = object_points[(2, 2)];
		let X3 = object_points[(0, 3)];
		let Y3 = object_points[(1, 3)];
		let Z3 = object_points[(2, 3)];

		let solutions = self.solven(
			rx0, ry0, rz0, X0, Y0, Z0, rx1, ry1, rz1, X1, Y1, Z1, rx2, ry2, rz2, X2, Y2, Z2,
		);

		// Map into poses with errors
		let solutions = solutions.into_iter().filter_map(|(R, t)| {
			let X3p = R[(0, 0)] * X3 + R[(0, 1)] * Y3 + R[(0, 2)] * Z3 + t[0];
			let Y3p = R[(1, 0)] * X3 + R[(1, 1)] * Y3 + R[(1, 2)] * Z3 + t[1];
			let Z3p = R[(2, 0)] * X3 + R[(2, 1)] * Y3 + R[(2, 2)] * Z3 + t[2];

			// Reject solutions behind the camera
			if Z3p <= 0.0 {
				return None;
			}

			let mu3p = intrinsics.cx + intrinsics.fx * X3p / Z3p;
			let mv3p = intrinsics.cy + intrinsics.fy * Y3p / Z3p;

			let reproj = (mu3p - (intrinsics.cx + intrinsics.fx * rx3 / rz3)).powi(2)
				+ (mv3p - (intrinsics.cy + intrinsics.fy * ry3 / rz3)).powi(2);

			Some(Pose3DWithError {
				pose: Pose3D::from_matrices(t, R),
				error: reproj,
			})
		});
		Some(PnPSolution::Multi(solutions.collect()))
	}
}

impl P3P {
	#[allow(unused_assignments)]
	fn solven(
		&self,
		rx0: f64,
		ry0: f64,
		rz0: f64,
		X0: f64,
		Y0: f64,
		Z0: f64,
		rx1: f64,
		ry1: f64,
		rz1: f64,
		X1: f64,
		Y1: f64,
		Z1: f64,
		rx2: f64,
		ry2: f64,
		rz2: f64,
		X2: f64,
		Y2: f64,
		Z2: f64,
	) -> Vec<(Matrix3<f64>, Matrix3x1<f64>)> {
		let mu0 = rx0;
		let mv0 = ry0;
		let mk0 = rz0;
		let mu1 = rx1;
		let mv1 = ry1;
		let mk1 = rz1;
		let mu2 = rx2;
		let mv2 = ry2;
		let mk2 = rz2;

		let distances = Matrix3x1::new(
			((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2) + (Z1 - Z2) * (Z1 - Z2)).sqrt(),
			((X0 - X2) * (X0 - X2) + (Y0 - Y2) * (Y0 - Y2) + (Z0 - Z2) * (Z0 - Z2)).sqrt(),
			((X0 - X1) * (X0 - X1) + (Y0 - Y1) * (Y0 - Y1) + (Z0 - Z1) * (Z0 - Z1)).sqrt(),
		);

		let cosines = Matrix3x1::new(
			mu1 * mu2 + mv1 * mv2 + mk1 * mk2,
			mu0 * mu2 + mv0 * mv2 + mk0 * mk2,
			mu0 * mu1 + mv0 * mv1 + mk0 * mk1,
		);

		let (n, lengths) = solve_for_lengths(&distances, &cosines);

		let mut out = Vec::new();

		for i in 0..n {
			let i = i as usize;
			let mut M_orig = Matrix3::zeros();

			M_orig[(0, 0)] = lengths[(0, i)] * mu0;
			M_orig[(0, 1)] = lengths[(0, i)] * mv0;
			M_orig[(0, 2)] = lengths[(0, i)] * mk0;

			M_orig[(1, 0)] = lengths[(1, i)] * mu1;
			M_orig[(1, 1)] = lengths[(1, i)] * mv1;
			M_orig[(1, 2)] = lengths[(1, i)] * mk1;

			M_orig[(2, 0)] = lengths[(2, i)] * mu2;
			M_orig[(2, 1)] = lengths[(2, i)] * mv2;
			M_orig[(2, 2)] = lengths[(2, i)] * mk2;

			let result = align(M_orig, X0, Y0, Z0, X1, Y1, Z1, X2, Y2, Z2);
			if let Some((translation, rotation)) = result {
				out.push((rotation, translation));
			}
		}

		out
	}
}

fn solve_for_lengths(distances: &Matrix3x1<f64>, cosines: &Matrix3x1<f64>) -> (u8, Matrix3x4<f64>) {
	let p = cosines[(0, 0)] * 2.0;
	let q = cosines[(1, 0)] * 2.0;
	let r = cosines[(2, 0)] * 2.0;

	if distances[(2, 0)].abs() < 1e-15 {
		return (0, Matrix3x4::zeros());
	}

	let inv_d22 = 1.0 / distances[(2, 0)].powi(2);
	let a = inv_d22 * distances[(0, 0)].powi(2);
	let b = inv_d22 * distances[(1, 0)].powi(2);

	let a2 = a * a;
	let b2 = b * b;
	let p2 = p * p;
	let q2 = q * q;
	let r2 = r * r;
	let pr = p * r;
	let pqr = q * pr;

	if p2 + q2 + r2 - pqr - 1.0 == 0.0 {
		return (0, Matrix3x4::zeros());
	}

	let ab = a * b;
	let a_2 = 2.0 * a;

	let A = -2.0 * b + b2 + a2 + 1.0 + ab * (2.0 - r2) - a_2;

	if A == 0.0 {
		return (0, Matrix3x4::zeros());
	}

	let a_4 = 4.0 * a;

	let B = q * (-2.0 * (ab + a2 + 1.0 - b) + r2 * ab + a_4) + pr * (b - b2 + ab);
	let C = q2 + b2 * (r2 + p2 - 2.0) - b * (p2 + pqr) - ab * (r2 + pqr)
		+ (a2 - a_2) * (2.0 + q2)
		+ 2.0;
	let D = pr * (ab - b2 + b) + q * ((p2 - 2.0) * b + 2.0 * (ab - a2) + a_4 - 2.0);
	let E = 1.0 + 2.0 * (b - a - ab) + b2 - b * p2 + a2;

	let b0 = b * (p2 * (a - 1.0 + b) + r2 * (a - 1.0 - b) + pqr - a * pqr).powi(2);

	if b0 == 0.0 {
		return (0, Matrix3x4::zeros());
	}

	let (n, root1, root2, root3, root4) = solve_deg4(A, B, C, D, E);
	if n == 0 {
		return (0, Matrix3x4::zeros());
	}

	let mut nb_solutions = 0;
	let r3 = r2 * r;
	let pr2 = p * r2;
	let r3q = r3 * q;

	let denom = 1.0 - b0;
	if denom.abs() < 1e-15 {
		return (0, Matrix3x4::zeros());
	}
	let inv_b0 = 1.0 / denom;

	let mut out = Matrix3x4::zeros();

	for x in [root1, root2, root3, root4] {
		if x <= 0.0 {
			continue;
		}

		let x2 = x * x;

		// What a monster
		let b1 = ((1.0 - a - b) * x2 + (q * a - q) * x + 1.0 - a + b)
			* (((r3 * (a2 + ab * (2.0 - r2) - a_2 + b2 - 2.0 * b + 1.0)) * x
				+ (r3q * (2.0 * (b - a2) + a_4 + ab * (r2 - 2.0) - 2.0)
					+ pr2 * (1.0 + a2 + 2.0 * (ab - a - b) + r2 * (b - b2) + b2)))
				* x2 + (r3
				* (q2 * (1.0 - 2.0 * a + a2) + r2 * (b2 - ab) - a_4 + 2.0 * (a2 - b2) + 2.0)
				+ r * p2 * (b2 + 2.0 * (ab - b - a) + 1.0 + a2)
				+ pr2 * q * (a_4 + 2.0 * (b - ab - a2) - 2.0 - r2 * b))
				* x + 2.0 * r3q * (a_2 - b - a2 + ab - 1.0)
				+ pr2 * (q2 - a_4 + 2.0 * (a2 - b2) + r2 * b + q2 * (a2 - a_2) + 2.0)
				+ p2 * (p * (2.0 * (ab - a - b) + a2 + b2 + 1.0)
					+ 2.0 * q * r * (b + a_2 - a2 - ab - 1.0)));

		if b1 <= 0.0 {
			continue;
		}

		let y = inv_b0 * b1;
		let v = x2 + y * y - x * y * r;

		if v <= 0.0 {
			continue;
		}

		let Z = distances[(2, 0)] / v.sqrt();
		let X = x * Z;
		let Y = y * Z;

		out[(0, nb_solutions)] = X;
		out[(1, nb_solutions)] = Y;
		out[(2, nb_solutions)] = Z;

		nb_solutions += 1;
	}

	(nb_solutions as u8, out)
}

fn align(
	M_end: Matrix3<f64>,
	X0: f64,
	Y0: f64,
	Z0: f64,
	X1: f64,
	Y1: f64,
	Z1: f64,
	X2: f64,
	Y2: f64,
	Z2: f64,
) -> Option<(Matrix3x1<f64>, Matrix3<f64>)> {
	let mut C_start = Matrix3x1::zeros();
	let mut C_end = Matrix3x1::zeros();

	for i in 0..3 {
		C_end[(i, 0)] = (M_end[(0, i)] + M_end[(1, i)] + M_end[(2, i)]) / 3.0;
	}

	C_start[(0, 0)] = (X0 + X1 + X2) / 3.0;
	C_start[(1, 0)] = (Y0 + Y1 + Y2) / 3.0;
	C_start[(2, 0)] = (Z0 + Z1 + Z2) / 3.0;

	let mut s = Matrix3::zeros();
	for i in 0..3 {
		s[(0, i)] = (X0 * M_end[(0, i)] + X1 * M_end[(1, i)] + X2 * M_end[(2, i)]) / 3.0
			- C_end[(i, 0)] * C_start[(0, 0)];
		s[(1, i)] = (Y0 * M_end[(0, i)] + Y1 * M_end[(1, i)] + Y2 * M_end[(2, i)]) / 3.0
			- C_end[(i, 0)] * C_start[(1, 0)];
		s[(2, i)] = (Z0 * M_end[(0, i)] + Z1 * M_end[(1, i)] + Z2 * M_end[(2, i)]) / 3.0
			- C_end[(i, 0)] * C_start[(2, 0)];
	}

	let mut Qs = Matrix4::<f64>::zeros();

	let Sxx = s[(0, 0)];
	let Sxy = s[(0, 1)];
	let Sxz = s[(0, 2)];
	let Syx = s[(1, 0)];
	let Syy = s[(1, 1)];
	let Syz = s[(1, 2)];
	let Szx = s[(2, 0)];
	let Szy = s[(2, 1)];
	let Szz = s[(2, 2)];

	let trace = Sxx + Syy + Szz;

	// Diagonal
	Qs[(0, 0)] = trace;
	Qs[(1, 1)] = Sxx - Syy - Szz;
	Qs[(2, 2)] = -Sxx + Syy - Szz;
	Qs[(3, 3)] = -Sxx - Syy + Szz;

	// Upper triangle
	Qs[(0, 1)] = Syz - Szy;
	Qs[(0, 2)] = Szx - Sxz;
	Qs[(0, 3)] = Sxy - Syx;

	Qs[(1, 2)] = Sxy + Syx;
	Qs[(1, 3)] = Szx + Sxz;
	Qs[(2, 3)] = Syz + Szy;

	// Symmetric lower triangle
	Qs[(1, 0)] = Qs[(0, 1)];
	Qs[(2, 0)] = Qs[(0, 2)];
	Qs[(3, 0)] = Qs[(0, 3)];
	Qs[(2, 1)] = Qs[(1, 2)];
	Qs[(3, 1)] = Qs[(1, 3)];
	Qs[(3, 2)] = Qs[(2, 3)];

	let (evs, U) = jacobi_4x4(&mut Qs);

	let mut i_ev = 0;
	let mut ev_max = evs[(i_ev, 0)];
	for i in 1..4 {
		if evs[(i, 0)] > ev_max {
			ev_max = evs[(i, 0)];
			i_ev = i;
		}
	}

	let mut q = Matrix4x1::zeros();
	for i in 0..4 {
		q[(i, 0)] = U[(i, i_ev)];
	}

	let q02 = q[0] * q[0];
	let q12 = q[1] * q[1];
	let q22 = q[2] * q[2];
	let q32 = q[3] * q[3];
	let q0_1 = q[0] * q[1];
	let q0_2 = q[0] * q[2];
	let q0_3 = q[0] * q[3];
	let q1_2 = q[1] * q[2];
	let q1_3 = q[1] * q[3];
	let q2_3 = q[2] * q[3];

	let mut R = Matrix3::zeros();
	R[(0, 0)] = q02 + q12 - q22 - q32;
	R[(0, 1)] = 2.0 * (q1_2 - q0_3);
	R[(0, 2)] = 2.0 * (q1_3 + q0_2);
	R[(1, 0)] = 2.0 * (q1_2 + q0_3);
	R[(1, 1)] = q02 + q22 - q12 - q32;
	R[(1, 2)] = 2.0 * (q2_3 - q0_1);
	R[(2, 0)] = 2.0 * (q1_3 - q0_2);
	R[(2, 1)] = 2.0 * (q2_3 + q0_1);
	R[(2, 2)] = q02 + q32 - q12 - q22;

	let mut t = Matrix3x1::zeros();
	for i in 0..3 {
		t[i] =
			C_end[i] - (R[(i, 0)] * C_start[0] + R[(i, 1)] * C_start[1] + R[(i, 2)] * C_start[2]);
	}

	Some((t, R))
}

fn jacobi_4x4(A: &mut Matrix4<f64>) -> (Matrix4x1<f64>, Matrix4<f64>) {
	let mut U = Matrix4::identity();
	let mut B = Matrix4x1::zeros();
	let mut Z = Matrix4x1::<f64>::zeros();

	B[(0, 0)] = A[(0, 0)];
	B[(1, 0)] = A[(1, 1)];
	B[(2, 0)] = A[(2, 2)];
	B[(3, 0)] = A[(3, 3)];
	let mut D = B.clone();

	for iter in 0..50 {
		let sum = A[1].abs() + A[2].abs() + A[3].abs() + A[6].abs() + A[7].abs() + A[11].abs();

		if sum == 0.0 {
			return (D, U);
		}

		let thresh = if iter < 3 { 0.2 * sum / 16.0 } else { 0.0 };

		for i in 0..3 {
			let mut pAij = unsafe { A.as_mut_ptr().add(5 * i + 1) };

			for j in (i + 1)..4 {
				let Aij = unsafe { *pAij };
				let eps_machine = 100.0 * Aij.abs();

				if iter > 3
					&& D[i].abs() + eps_machine == D[i].abs()
					&& D[j].abs() + eps_machine == D[j].abs()
				{
					unsafe {
						*pAij = 0.0;
					}
				} else if Aij.abs() > thresh {
					let hh = D[j] - D[i];
					let t = if hh.abs() + eps_machine == hh.abs() {
						Aij / hh
					} else {
						let theta = 0.5 * hh / Aij;
						let t = 1.0 / (theta.abs() + (1.0 + theta * theta).sqrt());

						if theta < 0.0 {
							-t
						} else {
							t
						}
					};

					let hh = t * Aij;
					Z[i] -= hh;
					Z[j] += hh;
					D[i] -= hh;
					D[j] += hh;
					unsafe {
						*pAij = 0.0;
					}

					let c = 1.0 / (1.0 + t * t).sqrt();
					let s = t * c;
					let tau = s / (1.0 + c);
					if i > 1 {
						for k in 0..=(i - 1) {
							let g = A[(k, i)];
							let h = A[(k, j)];
							A[(k, i)] = g - s * (h + g * tau);
							A[(k, j)] = h + s * (g - h * tau);
						}
					}
					if j > 1 {
						for k in (i + 1)..=(j - 1) {
							let g = A[(i, k)];
							let h = A[(k, j)];
							A[(i, k)] = g - s * (h + g * tau);
							A[(k, j)] = h + s * (g - h * tau);
						}
					}
					for k in 0..4 {
						let g = A[(i, k)];
						let h = A[(j, k)];
						A[(i, k)] = g - s * (h + g * tau);
						A[(j, k)] = h + s * (g - h * tau);
					}
					for k in 0..4 {
						let g = U[(k, i)];
						let h = U[(k, j)];
						U[(k, i)] = g - s * (h + g * tau);
						U[(k, j)] = h + s * (g - h * tau);
					}
				}

				unsafe {
					pAij = pAij.add(1);
				}
			}
		}
		for i in 0..4 {
			B[i] += Z[i];
		}

		D = B.clone();
		Z = Matrix4x1::zeros();
	}

	(D, U)
}

#[cfg(test)]
mod tests {
	use approx::assert_relative_eq;

	use super::*;

	#[test]
	fn test_jacobi_diagonal() {
		let mut A = Matrix4::zeros();
		A[(0, 0)] = 10.0;
		A[(1, 1)] = 7.0;
		A[(2, 2)] = 3.0;
		A[(3, 3)] = 1.0;

		let expected = Matrix4x1::new(10.0, 7.0, 3.0, 1.0);
		let (eig, vecs) = jacobi_4x4(&mut A);
		assert_relative_eq!(eig, expected, epsilon = 1e-12);

		for i in 0..4 {
			let v = vecs.row(i).transpose();
			let lhs = A * v;
			let rhs = v * eig[i];
			assert_relative_eq!(lhs, rhs, epsilon = 1e-10);
		}
	}
}
