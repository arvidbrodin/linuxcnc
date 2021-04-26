/*
 * This code ported from GNU Science Library v1.9, poly/solve_quadratic.c,
 * copyright (C) 1996, 1997, 1998, 1999, 2000 Brian Gough and released under
 * GPLv2 or later.
 *
 * Ported code copyright (C) 2021 Arvid Brodin.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#![allow(non_snake_case)]

pub struct Poly;

impl Poly {
	/* GSL code written to return +1.0 if num == -0.0. Rust's num.signum()
	 * returns -1.0 if num == -0.0, so we can't use that. */
	fn sgn(num: f64) -> f64 {
		if num >= -0.0 {
			return 1.0;
		}
		-1.0
	}

	pub fn solve_quadratic(a: f64, b: f64, c: f64) -> Vec<f64> {
		let mut res = Vec::new();

		// Handle linear case
		if a == 0.0 {
			if b == 0.0 {
				return res;
			} else {
				res.push(-c/b);
				return res;
			}
		}

		let disc = b.powi(2) - 4.0*a*c;

		if disc > 0.0 {
			if b == 0.0 {
				let r = (0.5*disc.sqrt()/a).abs();
				res.push(-r);
				res.push(r);
				return res;
			}

			let temp = -0.5*(b + Self::sgn(b)*disc.sqrt());
			let r1 = temp/a;
			let r2 = c/temp;

			if r1 < r2 {
				res.push(r1);
				res.push(r2);
			} else {
				res.push(r2);
				res.push(r1);
			}

			return res;
		}

		if disc == 0.0 {
			res.push(-0.5*b/a);
			res.push(-0.5*b/a);
			return res;
		}

		// Discriminant < 0.0; no roots
		return res;
	}
}
