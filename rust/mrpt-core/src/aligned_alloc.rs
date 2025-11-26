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

//! Aligned memory allocation
//!
//! This module provides functions for allocating memory with specific alignment requirements.

use std::alloc::{alloc_zeroed, dealloc, Layout};
use std::ptr;

/// Allocate aligned memory and initialize to zero
///
/// # Safety
/// The returned pointer must be freed with `aligned_free`
pub unsafe fn aligned_calloc(bytes: usize, alignment: usize) -> *mut u8 {
    if bytes == 0 {
        return ptr::null_mut();
    }

    // Adjust size to be a multiple of alignment
    let adjusted_size = if alignment != 0 && bytes % alignment != 0 {
        ((bytes / alignment) + 1) * alignment
    } else {
        bytes
    };

    match Layout::from_size_align(adjusted_size, alignment) {
        Ok(layout) => {
            let ptr = alloc_zeroed(layout);
            if ptr.is_null() {
                ptr::null_mut()
            } else {
                ptr
            }
        }
        Err(_) => ptr::null_mut(),
    }
}

/// Allocate aligned memory
///
/// # Safety
/// The returned pointer must be freed with `aligned_free`
pub unsafe fn aligned_malloc(size: usize, alignment: usize) -> *mut u8 {
    if size == 0 {
        return ptr::null_mut();
    }

    // Adjust size to be a multiple of alignment
    let adjusted_size = if alignment != 0 && size % alignment != 0 {
        ((size / alignment) + 1) * alignment
    } else {
        size
    };

    match Layout::from_size_align(adjusted_size, alignment) {
        Ok(layout) => {
            let ptr = std::alloc::alloc(layout);
            if ptr.is_null() {
                ptr::null_mut()
            } else {
                ptr
            }
        }
        Err(_) => ptr::null_mut(),
    }
}

/// Free memory allocated with aligned_malloc or aligned_calloc
///
/// # Safety
/// The pointer must have been allocated with `aligned_malloc` or `aligned_calloc`
/// with the same size and alignment parameters.
pub unsafe fn aligned_free(ptr: *mut u8, size: usize, alignment: usize) {
    if ptr.is_null() {
        return;
    }

    // Adjust size to match what was allocated
    let adjusted_size = if alignment != 0 && size % alignment != 0 {
        ((size / alignment) + 1) * alignment
    } else {
        size
    };

    if let Ok(layout) = Layout::from_size_align(adjusted_size, alignment) {
        dealloc(ptr, layout);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aligned_malloc() {
        unsafe {
            let ptr = aligned_malloc(64, 16);
            assert!(!ptr.is_null());
            assert_eq!(ptr as usize % 16, 0); // Check alignment
            aligned_free(ptr, 64, 16);
        }
    }

    #[test]
    fn test_aligned_calloc() {
        unsafe {
            let ptr = aligned_calloc(128, 32);
            assert!(!ptr.is_null());
            assert_eq!(ptr as usize % 32, 0); // Check alignment

            // Verify zeroed memory
            for i in 0..128 {
                assert_eq!(*ptr.add(i), 0);
            }

            aligned_free(ptr, 128, 32);
        }
    }

    #[test]
    fn test_aligned_malloc_size_adjustment() {
        unsafe {
            // Size not multiple of alignment - should be adjusted
            let ptr = aligned_malloc(65, 16);
            assert!(!ptr.is_null());
            assert_eq!(ptr as usize % 16, 0);
            aligned_free(ptr, 65, 16);
        }
    }
}
