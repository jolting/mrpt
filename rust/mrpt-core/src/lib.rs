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

//! # MRPT Core Library - Rust Implementation
//!
//! This crate provides the core functionality of the Mobile Robot Programming Toolkit (MRPT),
//! reimplemented in Rust for improved safety and performance.
//!
//! ## Modules
//!
//! - [`clock`] - Time and clock utilities
//! - [`exceptions`] - Exception handling and error types
//! - [`mod@format`] - String formatting utilities
//! - [`bits`] - Bit manipulation utilities

#![cfg_attr(not(feature = "std"), no_std)]
#![warn(missing_docs)]
#![warn(rust_2018_idioms)]

pub mod clock;
pub mod exceptions;
pub mod format;
pub mod bits;
pub mod cpu;
pub mod aligned_alloc;
pub mod demangle;
pub mod crc;
pub mod base64;
pub mod wrap2pi;
pub mod string_utils;
pub mod worker_threads;
pub mod containers;
pub mod env;

#[cfg(feature = "ffi")]
pub mod ffi;

pub use clock::{Clock, ClockSource};
pub use exceptions::{MrptError, MrptResult};
