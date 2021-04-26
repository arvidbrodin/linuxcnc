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
 *
 *
 * Description: Interface for calling the methods of Smooth1d from C.
 */

pub mod segment;
pub mod poly;
pub mod smooth1d;

use crate::smooth1d::Smooth1d;
use std::os::raw;

#[no_mangle]
pub extern "C" fn smooth1d_new(jerk: raw::c_double) -> *mut Smooth1d {
	Box::into_raw(Box::new(Smooth1d::new(Some(jerk))))
}

#[no_mangle]
pub extern "C" fn smooth1d_free(ptr: *mut Smooth1d) {
	if ptr.is_null() {
		return;
	}
	unsafe {
		// Consume ptr and let the Box go out of scope
		Box::from_raw(ptr);
	}
}

#[no_mangle]
pub extern "C" fn smooth1d_reset_pos(ptr: *mut Smooth1d, new_pos: raw::c_double) -> raw::c_int {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&mut *ptr
	};
	let res = smooth1d.reset_pos(new_pos);
	if res.is_err() {
		return -1;
	}
	return 0;
}

#[no_mangle]
pub extern "C" fn smooth1d_replan(ptr: *mut Smooth1d, s_target: raw::c_double, v_limit: raw::c_double, a_limit: raw::c_double) {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&mut *ptr
	};
	smooth1d.replan(s_target, v_limit, a_limit);
}

#[no_mangle]
pub extern "C" fn smooth1d_get_pos_cmd(ptr: *const Smooth1d) -> raw::c_double {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&*ptr
	};
	smooth1d.get_pos_cmd()
}

#[no_mangle]
pub extern "C" fn smooth1d_get_vel_limit(ptr: *const Smooth1d) -> raw::c_double {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&*ptr
	};
	smooth1d.get_vel_limit()
}

#[no_mangle]
pub extern "C" fn smooth1d_stop(ptr: *mut Smooth1d) {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&mut *ptr
	};
	smooth1d.stop();
}

#[no_mangle]
pub extern "C" fn smooth1d_update(ptr: *mut Smooth1d, dt: raw::c_double) {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&mut *ptr
	};
	smooth1d.update(dt);
}

#[no_mangle]
pub extern "C" fn smooth1d_get_pos(ptr: *const Smooth1d) -> raw::c_double {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&*ptr
	};
	smooth1d.get_state().0
}

#[no_mangle]
pub extern "C" fn smooth1d_get_vel(ptr: *const Smooth1d) -> raw::c_double {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&*ptr
	};
	smooth1d.get_state().1
}

#[no_mangle]
pub extern "C" fn smooth1d_get_acc(ptr: *const Smooth1d) -> raw::c_double {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&*ptr
	};
	smooth1d.get_state().2
}

#[no_mangle]
pub extern "C" fn smooth1d_is_active(ptr: *const Smooth1d) -> raw::c_int {
	let smooth1d = unsafe {
		assert!(!ptr.is_null());
		&*ptr
	};
	if smooth1d.is_active() {
		return 1;
	}
	return 0;
}
