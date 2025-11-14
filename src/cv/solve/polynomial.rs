#![allow(non_snake_case)]

// No idea why this triggers this lint
#[allow(unused_assignments)]
pub fn solve_deg4(a: f64, b: f64, c: f64, d: f64, e: f64) -> (u8, f64, f64, f64, f64) {
	if a == 0.0 {
		let (n, x1, x2, x3) = solve_deg3(b, c, d, e);
		return (n, x1, x2, x3, 0.0);
	}

	let inv_a = 1.0 / a;
	let b = b * inv_a;
	let c = c * inv_a;
	let d = d * inv_a;
	let e = e * inv_a;
	let b2 = b * b;
	let bc = b * c;
	let b3 = b2 * b;

	let (n, r0, _, _) = solve_deg3(1.0, -c, d * b - 4.0 * e, 4.0 * c * e - d * d - b2 * e);
	if n == 0 {
		return (0, 0.0, 0.0, 0.0, 0.0);
	}

	let R2 = 0.25 * b2 - c + r0;
	if R2 < 0.0 {
		return (0, 0.0, 0.0, 0.0, 0.0);
	}

	let R = R2.sqrt();
	let inv_R = 1.0 / R;

	let mut nb_real_roots = 0;

	let mut D2 = 0.0;
	let mut E2 = 0.0;

	if R < 10.0e-12 {
		let temp = r0 * r0 - 4.0 * e;
		if temp < 0.0 {
			D2 = -1.0;
			E2 = -1.0;
		} else {
			let sqrt_temp = temp.sqrt();
			D2 = 0.75 * b2 - 2.0 * c + 2.0 * sqrt_temp;
			E2 = D2 - 4.0 * sqrt_temp;
		}
	} else {
		let u = 0.75 * b2 - 2.0 * c - R2;
		let v = 0.25 * inv_R * (4.0 * bc - 8.0 * d - b3);
		D2 = u + v;
		E2 = u - v;
	}

	let b_4 = 0.25 * b;
	let R_2 = 0.5 * R;

	let mut x0 = 0.0;
	let mut x1 = 0.0;
	let mut x2 = 0.0;
	let mut x3 = 0.0;

	if D2 >= 0.0 {
		let D = D2.sqrt();
		nb_real_roots = 2;
		let D_2 = D / 2.0;
		x0 = R_2 + D_2 - b_4;
		x1 = R_2 - D_2 - b_4;
		// x1 = x0 - D;
	}

	if E2 >= 0.0 {
		let E = E2.sqrt();
		let E_2 = E / 2.0;
		if nb_real_roots == 0 {
			x0 = -R_2 + E_2 - b_4;
			x1 = -R_2 - E_2 - b_4;
			// x1 = x0 - E;
			nb_real_roots = 2;
		} else {
			x2 = -R_2 + E_2 - b_4;
			x3 = -R_2 - E_2 - b_4;
			// x1 = x0 - E;
			nb_real_roots = 4;
		}
	}

	(nb_real_roots, x0, x1, x2, x3)
}

pub fn solve_deg3(a: f64, b: f64, c: f64, d: f64) -> (u8, f64, f64, f64) {
	if a == 0.0 {
		if b == 0.0 {
			if c == 0.0 {
				return (0, 0.0, 0.0, 0.0);
			}

			return (1, -d / c, 0.0, 0.0);
		}

		let (n, x1, x2) = solve_deg2(b, c, d);
		return (n, x1, x2, 0.0);
	}

	let inv_a = 1.0 / a;
	let b_a = inv_a * b;
	let b_a2 = b_a * b_a;
	let c_a = inv_a * c;
	let d_a = inv_a * d;

	let Q = (3.0 * c_a - b_a2) / 9.0;
	let R = (9.0 * b_a * c_a - 27.0 * d_a - 2.0 * b_a * b_a2) / 54.0;
	let Q3 = Q * Q * Q;
	let D = Q3 + R * R;
	let b_a_3 = b_a / 3.0;

	if Q == 0.0 {
		if R == 0.0 {
			let x = -b_a_3;
			return (3, x, x, x);
		}

		let cube_root = (2.0 * R).cbrt();
		return (1, cube_root - b_a_3, 0.0, 0.0);
	}

	if D <= 0.0 {
		let theta = (R / (-Q3).sqrt()).acos();
		let sqrt_Q = (-Q).sqrt();
		let x1 = 2.0 * sqrt_Q * (theta / 3.0).cos() - b_a_3;
		let x2 = 2.0 * sqrt_Q * ((theta + 2.0 * std::f64::consts::PI) / 3.0).cos() - b_a_3;
		let x3 = 2.0 * sqrt_Q * ((theta + 4.0 * std::f64::consts::PI) / 3.0).cos() - b_a_3;

		return (3, x1, x2, x3);
	}

	let mut AD = 0.0;
	let mut BD = 0.0;
	let R_abs = R.abs();
	if R_abs > f64::EPSILON {
		AD = (R_abs + D.sqrt()).cbrt();
		AD = if R >= 0.0 { AD } else { -AD };

		BD = -Q / AD;
	}

	let x = AD + BD - b_a_3;
	(1, x, 0.0, 0.0)
}

pub fn solve_deg2(a: f64, b: f64, c: f64) -> (u8, f64, f64) {
	let delta = b * b - 4.0 * a * c;
	if delta < 0.0 {
		return (0, 0.0, 0.0);
	}

	let inv_2a = 0.5 / a;

	if delta == 0.0 {
		let x = -b * inv_2a;

		return (1, x, x);
	}

	let sqrt_delta = delta.sqrt();
	let x1 = (-b + sqrt_delta) * inv_2a;
	let x2 = (-b - sqrt_delta) * inv_2a;
	(2, x1, x2)
}
