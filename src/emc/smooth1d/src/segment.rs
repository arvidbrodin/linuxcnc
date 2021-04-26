/*
 * Copyright (c) 2021 Arvid Brodin
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

pub struct Segment {
	initvals: Vec<f64>,
	duration: f64,
	padto: usize,		// Return state of at least this length
}

impl Segment {
	pub fn new(initvals: &[f64], duration: f64, padto: usize) -> Result<Self, ()> {
		if duration < 0.0 {
			return Err(());
		}
		let seg = Self {
			initvals: initvals.to_vec(),
			duration: duration,
			padto: padto,
		};
//		seg.print();
		Ok(seg)
	}

	/*
	 * Treat initvals as coefficients of the terms in integrals of some
	 * derivative of position - that is, given
	 *
	 * s0 = initvals[2] (position)
	 * v0 = initvals[1] (velocity)
	 * a0 = initvals[0] (acceleration)
	 *
	 * return state at time t according to
	 *
	 * s = 1/6*j0*t³ + 1/2*a0*t² + v0*t + s0
	 * v = 1/2*j0*t² + a0*t + v0
	 * a = j0*t + a0
	 * j = j0
	 */
	pub fn get_state_at(&self, t: f64) -> Result<Vec<f64>, ()> {
		if t < 0.0 {
			eprintln!("Bug: get_state_at(): t < 0.0 (t = {})", t);
			return Err(());
		}
		if t > self.duration {
			eprintln!("Bug: get_state_at(): t > duration (t = {}, duration = {})", t, self.duration);
			return Err(());
		}

		let mut terms = Vec::new();
		let mut state = vec![0.0; self.padto - self.initvals.len()];
		for initval in &self.initvals {
			let mut val = 0.0;
			let degree = terms.len();
			for n in 0..degree {
				terms[n] *= t/(degree - n) as f64;
				val += terms[n];
			}
			terms.push(*initval);
			val += *initval;
			state.push(val);
		}

		Ok(state)
	}

	pub fn get_end_state(&self) -> Vec<f64> {
		/* Should be safe to call unwrap() here since we know duration
		 * is non-negative from the constructor's check, and duration
		 * cannot be shorter than itself!
		 */
		self.get_state_at(self.duration).unwrap()
	}

	pub fn get_duration(&self) -> f64 {
		self.duration
	}

	pub fn print(&self) {
		eprintln!("Segment: duration {}", self.duration);
		eprintln!("   Initvals: {:?}", self.initvals);
		eprintln!("   Endstate: {:?}", self.get_end_state());
	}
}
