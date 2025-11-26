// Mobile Robot Programming Toolkit (MRPT)
// https://www.mrpt.org/
//
// Copyright (c) 2005-2024, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// Released under BSD License. See: https://www.mrpt.org/License

//! FFI (Foreign Function Interface) bridge for C++ interoperability
//!
//! This module provides C-compatible interfaces to allow the C++ codebase
//! to call into the Rust implementation.

use crate::clock::{Clock, ClockSource, MrptTimePoint};
use std::os::raw::{c_char, c_int};

/// C-compatible clock source enum
#[repr(C)]
pub enum CClockSource {
    Realtime = 0,
    Monotonic = 1,
    Simulated = 2,
}

impl From<CClockSource> for ClockSource {
    fn from(source: CClockSource) -> Self {
        match source {
            CClockSource::Realtime => ClockSource::Realtime,
            CClockSource::Monotonic => ClockSource::Monotonic,
            CClockSource::Simulated => ClockSource::Simulated,
        }
    }
}

impl From<ClockSource> for CClockSource {
    fn from(source: ClockSource) -> Self {
        match source {
            ClockSource::Realtime => CClockSource::Realtime,
            ClockSource::Monotonic => CClockSource::Monotonic,
            ClockSource::Simulated => CClockSource::Simulated,
        }
    }
}

/// Get current time using the active clock source
#[no_mangle]
pub extern "C" fn mrpt_clock_now() -> i64 {
    Clock::now()
}

/// Get current time as double
#[no_mangle]
pub extern "C" fn mrpt_clock_now_double() -> f64 {
    Clock::now_double()
}

/// Convert double to time point
#[no_mangle]
pub extern "C" fn mrpt_clock_from_double(t: f64) -> i64 {
    Clock::from_double(t)
}

/// Convert time point to double
#[no_mangle]
pub extern "C" fn mrpt_clock_to_double(t: i64) -> f64 {
    Clock::to_double(t)
}

/// Set the active clock source
#[no_mangle]
pub extern "C" fn mrpt_clock_set_active(source: CClockSource) {
    Clock::set_active_clock(source.into());
}

/// Get the active clock source
#[no_mangle]
pub extern "C" fn mrpt_clock_get_active() -> CClockSource {
    Clock::get_active_clock().into()
}

/// Set simulated time
#[no_mangle]
pub extern "C" fn mrpt_clock_set_simulated_time(time: u64) {
    Clock::set_simulated_time(time);
}

/// Get simulated time
#[no_mangle]
pub extern "C" fn mrpt_clock_get_simulated_time() -> u64 {
    Clock::get_simulated_time()
}

/// Reset monotonic to realtime epoch
#[no_mangle]
pub extern "C" fn mrpt_clock_reset_monotonic_epoch() -> u64 {
    Clock::reset_monotonic_to_real_time_epoch()
}

/// Reverse bytes of a 16-bit value
#[no_mangle]
pub extern "C" fn mrpt_reverse_bytes_u16(value: u16) -> u16 {
    value.swap_bytes()
}

/// Reverse bytes of a 32-bit value
#[no_mangle]
pub extern "C" fn mrpt_reverse_bytes_u32(value: u32) -> u32 {
    value.swap_bytes()
}

/// Reverse bytes of a 64-bit value
#[no_mangle]
pub extern "C" fn mrpt_reverse_bytes_u64(value: u64) -> u64 {
    value.swap_bytes()
}

/// Reverse bytes of a float
#[no_mangle]
pub extern "C" fn mrpt_reverse_bytes_f32(value: f32) -> f32 {
    f32::from_bits(value.to_bits().swap_bytes())
}

/// Reverse bytes of a double
#[no_mangle]
pub extern "C" fn mrpt_reverse_bytes_f64(value: f64) -> f64 {
    f64::from_bits(value.to_bits().swap_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ffi_clock() {
        let t1 = mrpt_clock_now();
        let t2 = mrpt_clock_now();
        assert!(t2 >= t1);
    }

    #[test]
    fn test_ffi_clock_double_conversion() {
        let original = 123456789i64;
        let as_double = mrpt_clock_to_double(original);
        let back = mrpt_clock_from_double(as_double);
        assert_eq!(original, back);
    }

    #[test]
    fn test_ffi_reverse_bytes() {
        assert_eq!(mrpt_reverse_bytes_u16(0x1234), 0x3412);
        assert_eq!(mrpt_reverse_bytes_u32(0x12345678), 0x78563412);
    }
}
