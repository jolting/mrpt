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

// ========== Format functions ==========

/// Format a string using vsnprintf-style formatting
/// Returns the length of the formatted string (excluding null terminator)
/// or -1 on error.
///
/// # Safety
/// This function is unsafe because it dereferences raw pointers.
/// The caller must ensure that:
/// - `fmt` is a valid null-terminated string
/// - `buffer` points to a valid buffer of at least `buffer_len` bytes
#[no_mangle]
pub unsafe extern "C" fn mrpt_format_vsnprintf(
    buffer: *mut c_char,
    buffer_len: usize,
    fmt: *const c_char,
    args: *mut std::ffi::c_void,
) -> c_int {
    // This is a bridge function - in practice, we'll use platform-specific vsnprintf
    // For now, return error to indicate this should use native implementation
    -1
}

/// Get the length needed for a formatted string
///
/// # Safety
/// This function is unsafe because it dereferences raw pointers.
#[no_mangle]
pub unsafe extern "C" fn mrpt_format_get_length(
    fmt: *const c_char,
    args: *mut std::ffi::c_void,
) -> c_int {
    -1
}

// ========== Exception functions ==========

/// Create an exception message with location info
/// 
/// # Safety
/// This function is unsafe because it dereferences raw pointers.
/// The caller must ensure all pointers are valid null-terminated strings.
#[no_mangle]
pub unsafe extern "C" fn mrpt_exception_line_msg(
    msg: *const c_char,
    filename: *const c_char,
    line: u32,
    function_name: *const c_char,
    out_buffer: *mut c_char,
    buffer_len: usize,
) -> c_int {
    if msg.is_null() || filename.is_null() || function_name.is_null() || out_buffer.is_null() {
        return -1;
    }

    let msg_str = match std::ffi::CStr::from_ptr(msg).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    let filename_str = match std::ffi::CStr::from_ptr(filename).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };
    let function_str = match std::ffi::CStr::from_ptr(function_name).to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };

    let formatted = format!("{}:{}: [{}] {}\n", filename_str, line, function_str, msg_str);
    let formatted_bytes = formatted.as_bytes();
    
    if formatted_bytes.len() >= buffer_len {
        return -1; // Buffer too small
    }

    std::ptr::copy_nonoverlapping(
        formatted_bytes.as_ptr(),
        out_buffer as *mut u8,
        formatted_bytes.len(),
    );
    // Add null terminator
    *out_buffer.add(formatted_bytes.len()) = 0;

    formatted_bytes.len() as c_int
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
