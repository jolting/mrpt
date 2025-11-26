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

//! # MRPT Math Library - Rust Implementation
//!
//! This crate provides mathematical utilities, geometric types, and matrix operations
//! for the Mobile Robot Programming Toolkit (MRPT).
//!
//! ## Modules
//!
//! - [`epsilon`] - Geometric epsilon for floating-point comparisons
//! - [`utils`] - Math utility functions
//! - [`point2d`] - 2D point types

#![cfg_attr(not(feature = "std"), no_std)]
#![warn(missing_docs)]
#![warn(rust_2018_idioms)]

pub mod epsilon;
pub mod utils;
pub mod point2d;

pub use epsilon::{get_epsilon, set_epsilon};
pub use point2d::{TPoint2D, TPoint2Df};
