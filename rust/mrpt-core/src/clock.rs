//                    _
//                   | |    Mobile Robot Programming Toolkit (MRPT)
// _ __ ___  _ __ _ __ | |_
//| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
//| | | | | | |  | |_) | |_
//|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
//               | |
//               |_|
//
// Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! Clock and time functionality for MRPT
//!
//! This module provides time-related utilities compatible with MRPT's TTimeStamp representation.
//! It supports multiple clock sources: Realtime, Monotonic, and Simulated.
//!
//! # Examples
//!
//! ```
//! use mrpt_core::clock::{Clock, ClockSource};
//!
//! // Get current time
//! let now = Clock::now();
//! let timestamp = Clock::now_double();
//!
//! // Change clock source
//! Clock::set_active_clock(ClockSource::Monotonic);
//! ```

use parking_lot::RwLock;
use std::sync::Arc;

/// Clock sources available for time measurement
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockSource {
    /// POSIX CLOCK_REALTIME - highest resolution available (typically < 1 microsecond)
    Realtime,
    /// POSIX CLOCK_MONOTONIC - only available on Linux systems
    Monotonic,
    /// Simulated time - manually controlled via set_simulated_time()
    Simulated,
}

/// Duration in 100-nanosecond units (compatible with MRPT TTimeStamp)
pub type MrptDuration = i64;

/// Time point in 100-nanosecond units since epoch (compatible with MRPT TTimeStamp)
pub type MrptTimePoint = i64;

/// State for clock management
struct ClockState {
    selected_clock: ClockSource,
    simulated_time: u64,
    monotonic_to_realtime_epoch: Option<MonotonicEpoch>,
}

#[allow(dead_code)]
struct MonotonicEpoch {
    monotonic_ns: u64,
    realtime_ns: u64,
    rt2mono_diff: u64,
}

lazy_static::lazy_static! {
    static ref CLOCK_STATE: Arc<RwLock<ClockState>> = Arc::new(RwLock::new(ClockState {
        selected_clock: ClockSource::Realtime,
        simulated_time: 0,
        monotonic_to_realtime_epoch: None,
    }));
}

/// Main clock interface for MRPT - compatible with C++ implementation
pub struct Clock;

impl Clock {
    /// Returns the current time using the currently selected clock source
    ///
    /// Performance: typically takes ~33 nanoseconds
    #[inline]
    pub fn now() -> MrptTimePoint {
        let state = CLOCK_STATE.read();
        match state.selected_clock {
            ClockSource::Realtime => Self::now_realtime(),
            ClockSource::Monotonic => {
                drop(state);
                Self::now_monotonic_impl()
            }
            ClockSource::Simulated => state.simulated_time as i64,
        }
    }

    /// Equivalent to `Clock::to_double(Clock::now())`
    ///
    /// Performance: typically takes ~38 nanoseconds
    #[inline]
    pub fn now_double() -> f64 {
        Self::to_double(Self::now())
    }

    /// Create a timestamp from its double representation
    #[inline]
    pub fn from_double(t: f64) -> MrptTimePoint {
        (t * 10_000_000.0) as i64
    }

    /// Converts a timestamp to a UNIX time_t-like number with fractional part
    ///
    /// Returns 0.0 for invalid (default/zero) time points
    #[inline]
    pub fn to_double(t: MrptTimePoint) -> f64 {
        if t == 0 {
            0.0
        } else {
            t as f64 / 10_000_000.0
        }
    }

    /// Changes the selected clock source
    ///
    /// Note: It is strongly recommended to call `set_simulated_time()` before
    /// setting the clock source to Simulated to ensure subsequent calls to `now()`
    /// return defined values.
    pub fn set_active_clock(source: ClockSource) {
        let mut state = CLOCK_STATE.write();
        state.selected_clock = source;
    }

    /// Returns the currently selected clock source
    pub fn get_active_clock() -> ClockSource {
        CLOCK_STATE.read().selected_clock
    }

    /// Set the simulated time (only effective when ClockSource::Simulated is active)
    ///
    /// # Arguments
    /// * `time` - Time in 100-nanosecond units
    pub fn set_simulated_time(time: u64) {
        let mut state = CLOCK_STATE.write();
        state.simulated_time = time;
    }

    /// Get the current simulated time
    pub fn get_simulated_time() -> u64 {
        CLOCK_STATE.read().simulated_time
    }

    /// Resynchronize monotonic and realtime clocks
    ///
    /// Returns the mismatch between the former and new epoch estimations in nanoseconds
    pub fn reset_monotonic_to_real_time_epoch() -> u64 {
        let mut state = CLOCK_STATE.write();
        
        let old_diff = state.monotonic_to_realtime_epoch
            .as_ref()
            .map(|e| e.rt2mono_diff)
            .unwrap_or(0);

        let realtime_ns = Self::get_realtime_ns();
        let monotonic_ns = Self::get_monotonic_ns();
        
        let rt2mono_diff = if realtime_ns >= monotonic_ns {
            realtime_ns - monotonic_ns
        } else {
            0
        };

        state.monotonic_to_realtime_epoch = Some(MonotonicEpoch {
            monotonic_ns,
            realtime_ns,
            rt2mono_diff,
        });

        old_diff.saturating_sub(rt2mono_diff).max(rt2mono_diff.saturating_sub(old_diff))
    }

    // Private helper methods

    #[cfg(target_os = "windows")]
    fn now_realtime() -> MrptTimePoint {
        use std::time::SystemTime;
        let duration = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap();
        
        // Convert to 100-nanosecond units + Windows epoch offset
        let ns100 = duration.as_secs() * 10_000_000 + duration.subsec_nanos() as u64 / 100;
        (ns100 + 116_444_736_000_000_000) as i64
    }

    #[cfg(not(target_os = "windows"))]
    fn now_realtime() -> MrptTimePoint {
        let ns100 = Self::get_realtime_ns() / 100;
        // Add UNIX to Windows epoch offset
        (ns100 + 116_444_736_000_000_000) as i64
    }

    #[cfg(not(target_os = "windows"))]
    fn get_realtime_ns() -> u64 {
        use std::time::SystemTime;
        let duration = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap();
        duration.as_secs() * 1_000_000_000 + duration.subsec_nanos() as u64
    }

    #[cfg(target_os = "windows")]
    fn get_realtime_ns() -> u64 {
        use std::time::SystemTime;
        let duration = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap();
        duration.as_secs() * 1_000_000_000 + duration.subsec_nanos() as u64
    }

    fn now_monotonic_impl() -> MrptTimePoint {
        let state = CLOCK_STATE.read();
        let monotonic_ns = Self::get_monotonic_ns();
        
        // Check if epoch is initialized
        if state.monotonic_to_realtime_epoch.is_none() {
            // Need to initialize epoch - drop the read lock first
            drop(state);
            Self::reset_monotonic_to_real_time_epoch();
            return Self::now_monotonic_impl();
        }

        let epoch = state.monotonic_to_realtime_epoch.as_ref().unwrap();
        let adjusted_ns = monotonic_ns + epoch.rt2mono_diff;
        
        // Convert to 100-nanosecond units + Windows epoch offset
        let ns100 = adjusted_ns / 100;
        (ns100 + 116_444_736_000_000_000) as i64
    }

    #[cfg(target_os = "linux")]
    fn get_monotonic_ns() -> u64 {
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        unsafe {
            libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
        }
        ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
    }

    #[cfg(not(target_os = "linux"))]
    fn get_monotonic_ns() -> u64 {
        // Fallback to realtime on non-Linux systems
        Self::get_realtime_ns()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_clock_now() {
        // Ensure we're using realtime for this test
        Clock::set_active_clock(ClockSource::Realtime);
        
        let t1 = Clock::now();
        std::thread::sleep(Duration::from_millis(50));
        let t2 = Clock::now();
        assert!(t2 > t1, "t2 ({}) should be greater than t1 ({})", t2, t1);
    }

    #[test]
    fn test_clock_double_conversion() {
        let original = 123456789_i64;
        let as_double = Clock::to_double(original);
        let back = Clock::from_double(as_double);
        assert_eq!(original, back);
    }

    #[test]
    fn test_simulated_time() {
        let original_source = Clock::get_active_clock();
        
        Clock::set_simulated_time(100_000_000);
        Clock::set_active_clock(ClockSource::Simulated);
        
        let t = Clock::now();
        assert_eq!(t, 100_000_000);
        
        Clock::set_simulated_time(200_000_000);
        let t2 = Clock::now();
        assert_eq!(t2, 200_000_000);
        
        Clock::set_active_clock(original_source);
    }

    #[test]
    fn test_clock_sources() {
        Clock::set_active_clock(ClockSource::Realtime);
        assert_eq!(Clock::get_active_clock(), ClockSource::Realtime);
        
        Clock::set_active_clock(ClockSource::Simulated);
        assert_eq!(Clock::get_active_clock(), ClockSource::Simulated);
    }
}
