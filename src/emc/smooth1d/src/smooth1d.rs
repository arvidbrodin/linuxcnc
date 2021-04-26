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

use std::collections::VecDeque;
use crate::segment::Segment;
use crate::poly::Poly;

const TINY_DURATION: f64 = 1e-12;
const CLOSE_ENOUGH: f64 = 1e-12;
const NOT_CLOSE_ENOUGH: f64 = 1e-6;

pub struct Smooth1d {
	jerk: Option<f64>,
	targets: Vec<f64>,	// Highest-derivative first: targets[0] is jerk.
				// acc limit used by stop()
				// vel & pos available to caller
				// pos used to zero inaccuracies at end of move.
	segments: VecDeque<Segment>,
	time: f64,
	state: Vec<f64>,	// Highest-derivative first: state[0] is jerk.
}

impl Smooth1d {
	pub fn new(jerk: Option<f64>) -> Self {
		let degree;
		if jerk.is_some() {
			degree = 3;
		} else {
			degree = 2;
		}
		Self {
			jerk: jerk,
			targets: Vec::new(),
			segments: VecDeque::new(),
			time: 0.0,
			state: vec![0.0; degree + 1],
		}
	}

	pub fn reset_pos(&mut self, new_pos: f64) -> Result<(), ()> {
		if self.is_active() {
			return Err(());
		}

		let pos_index = self.state.len() - 1;
		self.state[pos_index] = new_pos;
		Ok(())
	}

	pub fn replan(&mut self, s_target: f64, v_limit: f64, a_limit: f64) {
		let pos_index = self.state.len() - 1;
		if (s_target == self.state[pos_index]) &&
		   (v_limit == self.state[pos_index - 1]) &&
		   (a_limit == self.state[pos_index - 2]) {
			// No change
			return;
		}

		println!("Smooth1d::replan(), state {:?}, s_target {}, v_limit {}, a_limit {}", self.state, s_target, v_limit, a_limit);

		if v_limit <= 0.0 {
			eprintln!("Bug: replan() must be called with v_limit > 0.0.");
			return;
		}
		if a_limit <= 0.0 {
			eprintln!("Bug: replan() must be called with a_limit > 0.0.");
			return;
		}

		self.targets.clear();
		if let Some(jerk) = &self.jerk {
			self.targets.push(*jerk);
			if self.targets[0] <= 0.0 {
				// Auto-calc of jerk limit.
				// This is rather arbitrary but seems to produce reasonable results
				// Unit analysis checks out too :)
				self.targets[0] = a_limit.powi(2)/v_limit;
			}
		}
		self.targets.push(a_limit);  // Need to save acc for stop() calls
		self.targets.push(v_limit);  // Only needed for planning this move
		self.targets.push(s_target);

//		let now = std::time::Instant::now();

		self.time = 0.0;
		self.segments.clear();

		if self.targets.len() == 3 {
			// Acc-limited path
			let res = self.calc_path_2(&self.targets.clone(), s_target);
			if res.is_err() {
				eprintln!("Bug: calc_path_2 failed, stop():ing");
				self.stop();
				return;
			}
		} else /* self.targets.len == 4 */ {
			// Jerk-limited path
			let res = self.calc_path_3(&self.targets.clone(), s_target);
			if res.is_err() {
				eprintln!("calc_path_3 failed, stop():ing");
				self.stop();
				return;
			}
		}

//		let duration = now.elapsed();
//		println!("replan() done in {:.6} s", duration.as_secs() as f64 + duration.subsec_nanos() as f64 * 1e-9);
	}

	pub fn stop(&mut self) {
		if !self.is_active() {
			return;
		}
		eprintln!("Smooth1d::stop(), state {:?}", self.state);

		self.time = 0.0;
		self.segments.clear();

		if self.targets.len() == 3 {
			// Acc-limited path
			self.calc_path_1(0.0);
		} else {
			// Jerk-limited path
			let res = self.calc_path_2(&self.targets.clone(), 0.0);
			if res.is_err() {
				eprintln!("Bug: calc_path_2 failed during stop()");
				self.calc_path_1(0.0);
			}
		}

		let target_index = self.targets.len() - 1;
		if !self.segments.is_empty() {
			let end_state = self.segments.back().unwrap().get_end_state();

			// Accept whatever position we end up at as target
			self.targets[target_index] = *end_state.last().unwrap();
		} else {
			self.targets[target_index] = *self.state.last().unwrap();
		}
	}

	pub fn update(&mut self, dt: f64) {
		if dt < 0.0 {
			eprintln!("Bug: update() must be called with dt >= 0.0.");
			return;
		}
		if self.segments.is_empty() {
			// No movement planned - nothing to do
			return;
		}

		self.time += dt;
		while self.time > self.segments[0].get_duration() {
			let seg = self.segments.pop_front().unwrap();
			self.time -= seg.get_duration();
			if self.segments.is_empty() {
				// Zero out any accumulated inaccuracies
				self.state = vec![0.0; self.targets.len() - 1];
				self.state.push(*self.targets.last().unwrap());

				break;
			}
		}

		if !self.segments.is_empty() {
			self.state = self.segments[0].get_state_at(self.time).expect("Bug: update(): get_state_at() failed.");
		}
	}

	fn get_end_state(&self) -> Vec<f64> {
		if self.segments.is_empty() {
			return self.state.clone();
		}
		self.segments.back().unwrap().get_end_state()
	}

	fn calc_path_1(&mut self, v_target: f64) {
		let mut state = self.get_end_state();
		let v_diff = v_target - state[1];
		let a0 = v_diff.signum()*self.targets[0];
		let t0 = v_diff/a0;

		let degree = self.targets.len() - 1;
		if t0.abs() > TINY_DURATION {
			state[0] = a0;
			let seg = Segment::new(&state[..], t0, degree + 1).expect("Bug: calc_path_1(): negative segment length!");
			self.segments.push_back(seg);
		}
	}

	fn calc_path_2(&mut self, limits: &Vec<f64>, s_target: f64) -> Result<(), ()> {
		let mut state = self.get_end_state();
		let s_diff = s_target - state[2];
		let v0 = state[1];

		let v1_target = s_diff.signum()*limits[1];
		let v1_diff = v1_target - v0;

//		println!("calc_path_2(): s_diff = {}; v1_target = {}", s_diff, v1_target);

		let mut a0 = v1_diff.signum()*limits[0];
		let mut t0 = v1_diff/a0;

		let mut a2 = -v1_target.signum()*limits[0];
		let mut t2 = -v1_target/a2;

		let mut t1 = s_diff/v1_target + 0.5*v0.powi(2)/(a0*v1_target) - 0.5*v1_target/a0 + 0.5*v1_target/a2;

//		println!("t0 = {}; t1 = {}; t2 = {}; a0 = {}; a2 = {}", t0, t1, t2, a0, a2);

		if t1 < 0.0 {
			// Solve for t0 with t1 = 0 (v_target never reached)
			let mut x = v0/a0;
			let mut roots = Poly::solve_quadratic(1.0, 2.0*x, 0.5*v0*x/a0 - s_diff/a0);
//			eprintln!("Roots: {:?}; x = {}", roots, x);
			if roots[1] < 0.0 {
				// We got the initial acceleration wrong
				// Test: alim_shortened_move_novmax
				a0 = -a0;
				x = v0/a0;
				roots = Poly::solve_quadratic(1.0, 2.0*x, 0.5*v0*x/a0 - s_diff/a0);
				if roots[1] < 0.0 {
					eprintln!("Bug: calc_path_2: roots[1] < 0.0.");
					return Err(());
				}
			}

			t0 = roots[1];
			t1 = 0.0;
			t2 = t0 + x;
			if roots[0] >= 0.0 {
				/* If both roots are positive, that means this
				 is a replanned move that overshoots the new
				 target, i.e. it crosses the target during t0.
				 Thus s_diff will have effectively changed sign
				 during t0, so change sign of a2. */
				// Tests: alim_shortened_move_pos/neg
				// FIXME: Check againt v_target needed here!
				a2 = -a0;
			}
		}

		let degree = self.targets.len() - 1;
		if t0.abs() > TINY_DURATION {
			state[0] = a0;
			self.segments.push_back(Segment::new(&state[..], t0, degree + 1)?);
			state = self.segments.back().unwrap().get_end_state();
		}

		if t1.abs() > TINY_DURATION {
			state[1] = v1_target;
			self.segments.push_back(Segment::new(&state[1..], t1, degree + 1)?);
			state = self.segments.back().unwrap().get_end_state();
		}

		if t2.abs() > TINY_DURATION {
			state[0] = a2;
			self.segments.push_back(Segment::new(&state[..], t2, degree + 1)?);
		}

		// Check result
		let state;
		let state_backing;
		if !self.segments.is_empty() {
			state_backing = self.segments.back().unwrap().get_end_state();
			state = &state_backing;
		} else {
			state = &self.state;
		}

		if !(state[1].abs() < CLOSE_ENOUGH*100.0) {
			eprintln!("Bug: calc_path_2(): state[1] not zero! state = {:?}; targets = {:?}", state, self.targets);
			return Err(());
		}
		if !((state[2] - s_target).abs() < CLOSE_ENOUGH*100.0) {
			eprintln!("Bug: calc_path_2(): target not reached! state = {:?}; s_target = {}", state, s_target);
			return Err(());
		}

		Ok(())
	}

	fn iterate_path_newton(&mut self, limits: &Vec<f64>, mut v_state: f64, mut s_prev: f64, mut v_prev: f64, s_target: f64) -> Result<(), ()> {
		let start_time = std::time::Instant::now();
//		let mut iterations = 0;
		loop {
			let state = self.get_end_state();
			let residual = s_target - state[3];

			let duration = start_time.elapsed();
			let ns = duration.as_secs()*1000000000 + duration.subsec_nanos() as u64;
			if (residual.abs() < CLOSE_ENOUGH) || (ns > 100000) {
//				println!("{} iterations in {} µs; residual {}", iterations, ns/1000, residual);
				if residual.abs() > NOT_CLOSE_ENOUGH {
					return Err(());
				}
				return Ok(());
			}
//			iterations += 1;

			let slope = (state[3] - s_prev)/(v_state - v_prev);
			s_prev = state[3];
			v_prev = v_state;
			v_state += residual/slope;
			self.segments.clear();
			self.calc_path_2(limits, v_state)?;
			self.calc_path_2(limits, 0.0)?;
		}
	}


	fn calc_path_3(&mut self, limits: &Vec<f64>, s_target: f64) -> Result<(), ()> {
		let s_diff = s_target - self.state[3];
		let v3_target = s_diff.signum()*limits[2];

		// Do we reach v_target?
		self.calc_path_2(limits, v3_target)?;
		let coast_index = self.segments.len();
		self.calc_path_2(limits, 0.0)?;

		let state = self.get_end_state();
		let t3 = (s_target - state[3])/v3_target;
		if t3 >= 0.0 {
			if t3.abs() < TINY_DURATION {
				return Ok(());
			}

			// We reached v_target. Insert coasting segment.
			while self.segments.len() > coast_index {
				self.segments.pop_back();
			}
			let mut state = self.get_end_state();
			state[2] = v3_target;
			let degree = self.targets.len() - 1;
			self.segments.push_back(Segment::new(&state[2..], t3, degree + 1)?);
			self.calc_path_2(limits, 0.0)?;

			return Ok(());
		}

		// Ok, we know we won't reach v_target. Do we increase or decrease acc?

		let s_v3_target = state[3];  // Save pos for iteration start value

		self.segments.clear();
		self.calc_path_2(limits, 0.0)?;
		let state = self.get_end_state();
		let s_v_zero = state[3];  // Save for iteration start values
		if (s_target - state[3]).signum() == s_diff.signum() {
			// We increase acc.
			let j0 = s_diff.signum()*limits[0];

			// Do we reach a_lim?
			let v3_for_a_lim = (limits[1].powi(2) - 0.5*self.state[1].powi(2))/j0 + self.state[2];

			self.segments.clear();
			self.calc_path_2(limits, v3_for_a_lim)?;
			self.calc_path_2(limits, 0.0)?;

			let state = self.get_end_state();
			if (s_target - state[3]).signum() == s_diff.signum() {
				// Yes, we reach a_lim. Peak v between v3_for_a_lim and v3_target.
				return self.iterate_path_newton(limits, v3_for_a_lim, s_v3_target, v3_target, s_target);
			}

			// No, we don't reach a_lim. Peak v between 0 and v3_for_a_lim.
			return self.iterate_path_newton(limits, v3_for_a_lim, s_v_zero, 0.0, s_target);
		}

		// We need to reverse to reach s_target
//		todo!("We overhoot");

		// Crudely guess at -v3_target?

		self.calc_path_2(limits, -v3_target)?;
		self.calc_path_2(limits, 0.0)?;
		return self.iterate_path_newton(limits, -v3_target, s_v_zero, 0.0, s_target);
	}

	pub fn get_state(&self) -> (f64, f64, f64) {
		let pos_index = self.state.len() - 1;
		(self.state[pos_index], self.state[pos_index - 1], self.state[pos_index - 2])
	}

	pub fn get_pos_cmd(&self) -> f64 {
		if self.targets.len() < 1 {
			return 0.0;
		}
		let pos_index = self.targets.len() - 1;
		self.targets[pos_index]
	}

	pub fn get_vel_limit(&self) -> f64 {
		if self.targets.len() < 2 {
			return 0.0;
		}
		let vel_index = self.targets.len() - 2;
		self.targets[vel_index]
	}

	pub fn is_active(&self) -> bool {
		!self.segments.is_empty()
	}

	pub fn print(&self) {
		for seg in &self.segments {
			seg.print();
		}
	}
}


#[cfg(test)]
mod tests {
	struct LimitChecker {
		limit: f64,
		next_limit: f64,
	}

	impl LimitChecker {
		pub fn new() -> Self {
			Self {
				limit: std::f64::MAX,
				next_limit: std::f64::MAX,
			}
		}

		pub fn set_limit(&mut self, limit: f64) {
			self.next_limit = limit;
		}

		pub fn get_limit(&self) -> f64 {
			self.limit
		}

		pub fn is_violation(&mut self, value: f64) -> bool {
			let x = value.abs();
			if x <= self.next_limit {
				// Remove relaxation as soon as we're under the new limit
				self.limit = self.next_limit;
			}

			x > self.limit
		}
	}

	use super::Smooth1d;
	use super::NOT_CLOSE_ENOUGH;
	use std::io::Write;
	use crate::segment::Segment;

	enum ActionType {
		MoveTo((f64, f64, f64)),
		CheckAcc(f64),
		CheckVel(f64),
		CheckPos(f64),
		CheckState((f64, f64, f64)),
		SetState((f64, f64, f64, f64), (f64, f64, f64, f64)),
		Stop,
		Done,
	}

	struct Action {
		t: f64,
		action: ActionType,
	}

	fn val_eq(val1: f64, val2: f64) -> bool {
		(val1 - val2).abs() < NOT_CLOSE_ENOUGH
	}

	fn check_eq(time: f64, val: f64, goal: f64) -> Result<(), String> {
		if val_eq(val, goal) {
			return Ok(());
		}

		Err(format!("Time {:.3}: value {} differs from {}", time, val, goal))
	}

	fn check_states_eq(time: f64, state: (f64, f64, f64), goal: (f64, f64, f64)) -> Result<(), String> {
		if val_eq(state.0, goal.0) && val_eq(state.1, goal.1) && val_eq(state.2, goal.2) {
			return Ok(());
		}

		Err(format!("Time {:.3}: state {:?} differs from {:?}", time, state, goal))
	}

	fn write_state(orig_state: &Vec<f64>, file: &mut std::fs::File) {
		for val in orig_state.iter().rev() {
			write!(file, "{:.6} ", *val).expect("Could not write to file");
		}

		// Print dummy jerk to keep gnuplot happy
		if orig_state.len() < 4 {
			write!(file, "{:.6} ", 0.0).expect("Could not write to file");
		}
	}

	fn run_test(op_jerk: Option<f64>, actions: &[Action], test_name: &str) -> Result<(), String> {
		let mut file = std::fs::File::create(test_name.to_owned() + ".data").expect("Cannot create data file");

		let mut replans = Vec::new();

		let mut result = Ok(());

		let dt = 0.001;
		let tolerance_fact = 1.01;

		let mut smooth1d = Smooth1d::new(op_jerk);
		let mut t = 0.0;
		let mut s_prev = 0.0;
		let mut v_prev = 0.0;
		let mut a_prev = 0.0;
		let mut v_lim = LimitChecker::new();
		let mut a_lim = LimitChecker::new();
		let mut check_delay = 0;
		let mut action_index = 0;
		let mut done = false;
		loop {
			// Check limits
			let state = smooth1d.get_state();

			let v = (state.0 - s_prev)/dt;
			if result.is_ok() {
				if v_lim.is_violation(v) {
					result = Err(format!("Time {:.6}: velocity ({}) over limit ({})", t, v, v_lim.get_limit()));
				}
			}

			let a = (v - v_prev)/dt;
			if result.is_ok() {
				if a_lim.is_violation(a) {
					result = Err(format!("Time {:.6}: acceleration ({}) over limit ({})", t, a, a_lim.get_limit()));
				}
			}

			if smooth1d.targets.len() >= 4 && check_delay == 0 {
				let jlim = smooth1d.targets[0];
				let j = (a - a_prev)/dt;
				if result.is_ok() {
					if j.abs() > jlim*tolerance_fact {
						result = Err(format!("Time {:.6}: jerk ({}) over limit ({})", t, j, jlim*tolerance_fact));
					}
				}
			}
			if check_delay > 0 {
				check_delay -= 1;
			}

			s_prev = state.0;
			v_prev = v;
			a_prev = a;

			// Check specific values at specific times
			loop {
				let action = &actions[action_index];
				// Perform all actions up until (but not including) next update
				if action.t >= t + dt {
					break;
				}

				match action.action {
					ActionType::MoveTo((x, v, a)) => {
						smooth1d.replan(x, v, a);
						replans.push(action.t);
						v_lim.set_limit(v*tolerance_fact);
						a_lim.set_limit(a*tolerance_fact);
					},
					ActionType::Stop => {
						smooth1d.stop();
						replans.push(action.t);
					},
					ActionType::CheckAcc(acc) => {
						if result.is_ok() {
							let state = smooth1d.get_state();
							result = check_eq(action.t, state.2, acc);
						}
					},
					ActionType::CheckVel(vel) => {
						if result.is_ok() {
							let state = smooth1d.get_state();
							result = check_eq(action.t, state.1, vel);
						}
					},
					ActionType::CheckPos(pos) => {
						if result.is_ok() {
							let state = smooth1d.get_state();
							result = check_eq(action.t, state.0, pos);
						}
					},
					ActionType::CheckState(goal) => {
						if result.is_ok() {
							let state = smooth1d.get_state();
							result = check_states_eq(action.t, state, goal);
						}
					},
					ActionType::SetState(state, targets) => {
						smooth1d.state.clear();
						smooth1d.state.push(state.0); // jerk
						smooth1d.state.push(state.1); // acc
						a_prev = state.1; // - state.0*dt;
						smooth1d.state.push(state.2); // vel
						v_prev = state.2; // - state.1*dt;
						smooth1d.state.push(state.3); // pos
						s_prev = state.3; // - state.2*dt;

						smooth1d.targets.clear();
						smooth1d.targets.push(targets.0);
						smooth1d.targets.push(targets.1);
						a_lim.set_limit(targets.1*tolerance_fact);
						smooth1d.targets.push(targets.2);
						v_lim.set_limit(targets.2*tolerance_fact);
						smooth1d.targets.push(targets.3);

						// Add dummy segment for is_active() to return true
						smooth1d.segments.clear();
						smooth1d.segments.push_back(Segment::new(&smooth1d.state[..], 0.1, smooth1d.targets.len()).unwrap());

						check_delay = 3;
					},
					ActionType::Done => {
						if result.is_ok() {
							if smooth1d.is_active() {
								result = Err(format!("Time {:.3}: Planner still active", action.t));
							}
						}
						done = true;
						break;
					},
				}
				action_index += 1;
			}
			if done {
				break;
			}

			smooth1d.update(dt);

			// Plot state to file
			write!(file, "{:.6} ", t).expect("Could not write to file");
			write_state(&smooth1d.state, &mut file);
			writeln!(file, "").expect("Could not write to file");

			t += dt;
		}

		writeln!(file, "").expect("Could not write to file");
		writeln!(file, "").expect("Could not write to file");
		for t in replans {
			write!(file, "{:.6} ", t).expect("Could not write to file");
		}
		writeln!(file, "").expect("Could not write to file");

		result
	}

	// ***
	// *** Test acceleration-limited motion ***
	// ***

	// Zero-distance move
	#[test]
	fn alim_null_move() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.00, MAX_VEL, MAX_ACC)) },
			Action { t: 0.01, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_null_move")
	}

	// Long move that reach all the limits
	#[test]
	fn alim_move_all_limits_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.10, action: ActionType::CheckAcc(MAX_ACC) },
			Action { t: 0.301, action: ActionType::CheckState((0.02, MAX_VEL, 0.0)) },
			Action { t: 0.601, action: ActionType::CheckState((0.04, 0.0, 0.0)) },
			Action { t: 0.61, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_move_all_limits_pos")
	}
	#[test]
	fn alim_move_all_limits_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.10, action: ActionType::CheckAcc(-MAX_ACC) },
			Action { t: 0.301, action: ActionType::CheckState((-0.02, -MAX_VEL, 0.0)) },
			Action { t: 0.601, action: ActionType::CheckState((-0.04, 0.0, 0.0)) },
			Action { t: 0.61, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_move_all_limits_neg")
	}

	// Short, separate moves that reach a_max but not v_max
	#[test]
	fn alim_move_no_vmax_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.2;
		const MAX_ACC: f64 = 1.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.02, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::CheckState((0.02, 0.0, 0.0)) },
			Action { t: 0.31, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_move_no_vmax_pos")
	}
	#[test]
	fn alim_move_no_vmax_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.2;
		const MAX_ACC: f64 = 1.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.02, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::CheckState((-0.02, 0.0, 0.0)) },
			Action { t: 0.31, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_move_no_vmax_neg")
	}

	// Plan a move, then plan another one that continues in exactly the same way
	#[test]
	fn alim_continued_move_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.601, action: ActionType::CheckState((0.04, 0.0, 0.0)) },
			Action { t: 0.61, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_continued_move_pos")
	}
	#[test]
	fn alim_continued_move_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((-0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.601, action: ActionType::CheckState((-0.04, 0.0, 0.0)) },
			Action { t: 0.61, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_continued_move_neg")
	}

	// Interrupted move, higher velocity
	#[test]
	fn alim_inc_vel_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.06, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((0.06, MAX_VEL*1.5, MAX_ACC)) },
			Action { t: 0.41, action: ActionType::CheckVel(MAX_VEL*1.5) },
			Action { t: 0.80, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_inc_vel_pos")
	}

	// Interrupted move, higher velocity, doesn't reach vmax
	#[test]
	fn alim_inc_vel_novmax_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.05, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((0.05, MAX_VEL*1.5, MAX_ACC)) },
			Action { t: 0.70, action: ActionType::CheckPos(0.05) },
			Action { t: 0.71, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_inc_vel_novmax_pos")
	}

	// Interrupted move, lower velocity
	#[test]
	fn alim_dec_vel_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.05, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((0.05, MAX_VEL*0.5, MAX_ACC)) },
			Action { t: 1.00, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_dec_vel_pos")
	}

	// Shortened move - new command at s = 0.020 overshoots to 0.030 before going back to 0.025
	#[test]
	fn alim_shortened_move_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.05, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((0.025, MAX_VEL*0.3, MAX_ACC)) },
			Action { t: 0.80, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_shortened_move_pos")
	}
	#[test]
	fn alim_shortened_move_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.05, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((-0.025, MAX_VEL*0.3, MAX_ACC)) },
			Action { t: 0.80, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_shortened_move_neg")
	}

	// Reversed move - new command at s = 0.020, v > 0, with target that needs v < 0.
	#[test]
	fn alim_reversed_move_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.05, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((0.010, MAX_VEL*0.5, MAX_ACC)) },
			Action { t: 1.01, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_reversed_move_pos")
	}
	#[test]
	fn alim_reversed_move_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.05, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((-0.010, MAX_VEL*0.5, MAX_ACC)) },
			Action { t: 1.01, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_reversed_move_neg")
	}

	// Stop during positive velocity
	#[test]
	fn alim_stop_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.15, action: ActionType::Stop },
			Action { t: 0.35, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_stop_pos")
	}
	// Stop during negative velocity
	#[test]
	fn alim_stop_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.15, action: ActionType::Stop },
			Action { t: 0.35, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_stop_neg")
	}
	#[test]
	fn alim_stop_interrupted() -> Result<(), String> {
		const MAX_VEL: f64 = 0.2;
		const MAX_ACC: f64 = 0.15;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.25, MAX_VEL, MAX_ACC)) },
			Action { t: 2.00, action: ActionType::MoveTo((-0.10, MAX_VEL, MAX_ACC)) },
			Action { t: 3.00, action: ActionType::Stop },
			Action { t: 4.00, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_stop_interrupted")
	}

	#[test]
	fn alim_stop_accelerating() -> Result<(), String> {
//		const MAX_VEL: f64 = 0.03;
//		const MAX_ACC: f64 = 0.5;
		const MAX_VEL: f64 = 508.0;
		const MAX_ACC: f64 = 8493.603949533735;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-25.0, MAX_VEL, MAX_ACC)) },
			Action { t: 0.05, action: ActionType::MoveTo((  0.0, MAX_VEL, MAX_ACC)) },
			Action { t: 0.40, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_stop_accelerating")
	}

	// Shortened move - new command overshoots. Triggers bug in path_2 calc of v1_target & a0
	#[test]
	fn alim_shortened_move_novmax() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.05, MAX_VEL, MAX_ACC)) },
			Action { t: 0.15, action: ActionType::MoveTo((0.01, MAX_VEL, MAX_ACC)) },
			Action { t: 0.80, action: ActionType::Done },
		];
		run_test(None, &actions, "alim_shortened_move_novmax")
	}


	// ***
	// *** Test jerk-limited motion ***
	// ***

	#[test]
	fn jlim_stop_unknown() -> Result<(), String> {
		let state = (0.0, -508.0, -9.797000000000038, 8.800070608620487);
		let targets = (8493.603949533735, 508.0, 30.383333333333333, -500.27938693053187);
		let actions = [
			Action { t: 0.10, action: ActionType::SetState(state, targets) },
			Action { t: 0.11, action: ActionType::Stop },
			Action { t: 0.40, action: ActionType::Done },
		];
		run_test(Some(0.0), &actions, "jlim_stop_unknown")
	}

	#[test]
	#[ignore] // Better tested by alim_shortened_move_novmax
	fn jlim_negative_roots() -> Result<(), String> {
		let state = (8466.666666666666, 254.00000000000006, 3.8100000000000023, 4.0381);
		let targets = (8466.666666666666, 508.0, 30.48, 4.0381);
		let actions = [
			Action { t: 0.10, action: ActionType::SetState(state, targets) },
			Action { t: 0.10, action: ActionType::MoveTo((5.0, 30.48, 508.0)) },
			Action { t: 0.40, action: ActionType::Done },
		];
		run_test(Some(0.0), &actions, "jlim_negative_roots")
	}

	// Long move that reach all the limits
	#[test]
	fn jlim_move_all_limits_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		const JERK: f64 = 5.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.15, action: ActionType::CheckAcc(MAX_ACC) },
			Action { t: 0.351, action: ActionType::CheckState((0.02, MAX_VEL, 0.0)) },
			Action { t: 0.701, action: ActionType::CheckState((0.04, 0.0, 0.0)) },
			Action { t: 0.71, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_move_all_limits_pos")
	}
	#[test]
	fn jlim_move_all_limits_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		const JERK: f64 = 5.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.15, action: ActionType::CheckAcc(-MAX_ACC) },
			Action { t: 0.351, action: ActionType::CheckState((-0.02, -MAX_VEL, 0.0)) },
			Action { t: 0.701, action: ActionType::CheckState((-0.04, 0.0, 0.0)) },
			Action { t: 0.71, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_move_all_limits_neg")
	}

	// Long, separate moves that reach v_max but not a_max
	#[test]
	fn jlim_move_no_amax_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 10.0;
		const JERK: f64 = 62.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.02, MAX_VEL, MAX_ACC)) },
			Action { t: 0.141, action: ActionType::CheckState((0.01, MAX_VEL, 0.0)) },
			Action { t: 0.281, action: ActionType::CheckState((0.02, 0.0, 0.0)) },
			Action { t: 0.29, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_move_no_amax_pos")
	}
	#[test]
	fn jlim_move_no_amax_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 10.0;
		const JERK: f64 = 62.5;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.02, MAX_VEL, MAX_ACC)) },
			Action { t: 0.141, action: ActionType::CheckState((0.01, MAX_VEL, 0.0)) },
			Action { t: 0.281, action: ActionType::CheckState((0.02, 0.0, 0.0)) },
			Action { t: 0.29, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_move_no_amax_neg")
	}

	// Short, separate moves that reach a_max but not v_max
	#[test]
	fn jlim_move_no_vmax_pos() -> Result<(), String> {
		const TARGET: f64 = std::f64::consts::PI/100.0;
		const MAX_VEL: f64 = 0.2;
		const MAX_ACC: f64 = 1.0;
		const JERK: f64 = 10.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((TARGET, MAX_VEL, MAX_ACC)) },
			Action { t: 0.12, action: ActionType::CheckAcc(MAX_ACC) },
			Action { t: 0.35, action: ActionType::CheckAcc(-MAX_ACC) },
			Action { t: 0.48, action: ActionType::CheckState((TARGET, 0.0, 0.0)) },
			Action { t: 0.50, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_move_no_vmax_pos")
	}
	#[test]
	fn jlim_move_no_vmax_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.2;
		const MAX_ACC: f64 = 1.0;
		const JERK: f64 = 10.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.03, MAX_VEL, MAX_ACC)) },
			Action { t: 0.12, action: ActionType::CheckAcc(-MAX_ACC) },
			Action { t: 0.35, action: ActionType::CheckAcc(MAX_ACC) },
			Action { t: 0.48, action: ActionType::CheckState((-0.03, 0.0, 0.0)) },
			Action { t: 0.50, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_move_no_vmax_neg")
	}

	// Short, separate moves that reach neither v_max nor a_max
	#[test]
	fn jlim_move_no_vmax_nor_amax_pos() -> Result<(), String> {
		const MAX_VEL: f64 = 0.2;
		const MAX_ACC: f64 = 1.0;
		const JERK: f64 = 10.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.00315, MAX_VEL, MAX_ACC)) },
			Action { t: 0.25, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_move_no_vmax_nor_amax_pos")
	}

	#[test]
	fn jlim_move_no_vmax_nor_amax_neg() -> Result<(), String> {
		const MAX_VEL: f64 = 0.2;
		const MAX_ACC: f64 = 1.0;
		const JERK: f64 = 10.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((-0.0025, MAX_VEL, MAX_ACC)) },
			Action { t: 0.101, action: ActionType::CheckState((-0.00125, -0.025, 0.0)) },
			Action { t: 0.201, action: ActionType::CheckState((-0.0025, 0.0, 0.0)) },
			Action { t: 0.21, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_move_no_vmax_nor_amax_neg")
	}

	// Plan a move, then plan another one that continues in exactly the same way
	#[test]
	fn jlim_continued_move() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		const JERK: f64 = 5.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.35, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.701, action: ActionType::CheckState((0.04, 0.0, 0.0)) },
			Action { t: 0.71, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_continued_move")
	}

	// Plan a move, then plan another one that's ahead but requires reversing
	#[test]
	fn jlim_reverse_ahead() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		const JERK: f64 = 5.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.10, action: ActionType::MoveTo((0.002, MAX_VEL, MAX_ACC)) },
			Action { t: 0.71, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_reverse_ahead")
	}

	// Stop from v_max
	#[test]
	fn jlim_stop_at_vmax() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		const JERK: f64 = 5.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.35, action: ActionType::Stop },
			Action { t: 0.65, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_stop_at_vmax")
	}

	// Stop from a_max (not yet at v_max)
	#[test]
	fn jlim_stop_at_amax() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		const JERK: f64 = 5.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.15, action: ActionType::Stop },
			Action { t: 0.51, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_stop_at_amax")
	}

	// Stop from positive a, v (neither yet at max)
	#[test]
	fn jlim_stop_not_at_limits() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		const JERK: f64 = 5.0;
		let actions = [
			Action { t: 0.01, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.25, action: ActionType::Stop },
			Action { t: 0.62, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_stop_not_at_limits")
	}

	/*
	 * Stop from a = -a_max and v > 0 but small enough to pass v = 0
	 * before a = 0. This requires heading into negative v before stopping.
	 */
	#[test]
	fn jlim_stop_switch_v() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 0.5;
		const JERK: f64 = 5.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.35, action: ActionType::MoveTo((-0.04, MAX_VEL, MAX_ACC)) },
			Action { t: 0.58, action: ActionType::Stop },
			Action { t: 0.80, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_stop_switch_v")
	}


	// Interrupted moves at v_max: same v_max, same direction
	// Interrupted moves below v_max: same v_max, same direction
	// Interrupted moves at v_max: same v_max, other direction
	// Interrupted moves below v_max: same v_max, other direction
	// Interrupted moves at v_max: target too close (position overshoot)
	// Interrupted moves below v_max: target too close (position overshoot)

	#[test]
	fn jlim_complex() -> Result<(), String> {
		const MAX_VEL: f64 = 0.1;
		const MAX_ACC: f64 = 1.0;
		const JERK: f64 = 10.0;
		let actions = [
			Action { t: 0.00, action: ActionType::MoveTo((0.010, MAX_VEL, MAX_ACC)) },
			Action { t: 0.10, action: ActionType::MoveTo((0.020, MAX_VEL, MAX_ACC)) },
			Action { t: 0.20, action: ActionType::MoveTo((0.030, MAX_VEL, MAX_ACC)) },
			Action { t: 0.30, action: ActionType::MoveTo((0.040, MAX_VEL, MAX_ACC)) },
			Action { t: 0.50, action: ActionType::MoveTo((0.030, MAX_VEL, MAX_ACC)) },
//			Action { t: 0.50, action: ActionType::Stop },
			Action { t: 0.99, action: ActionType::Done },
		];
		run_test(Some(JERK), &actions, "jlim_complex")
	}
}
