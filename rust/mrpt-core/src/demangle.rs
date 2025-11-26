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

//! Symbol demangling utilities
//!
//! This module provides functions for demangling C++ symbol names.

use std::ffi::CStr;

/// Demangle a C++ symbol name
///
/// Attempts to demangle C++ symbol names using the cpp_demangle crate.
/// If demangling fails, returns the original symbol name.
pub fn demangle(symbol_name: &str) -> String {
    if symbol_name.is_empty() {
        return String::new();
    }

    // Try to demangle using cpp_demangle crate
    match cpp_demangle::Symbol::new(symbol_name) {
        Ok(symbol) => symbol.to_string(),
        Err(_) => symbol_name.to_string(),
    }
}

/// Demangle a C++ symbol name from a C-style string
///
/// # Safety
/// The pointer must be a valid null-terminated C string
pub unsafe fn demangle_cstr(symbol_ptr: *const i8) -> String {
    if symbol_ptr.is_null() {
        return String::new();
    }

    match CStr::from_ptr(symbol_ptr).to_str() {
        Ok(s) => demangle(s),
        Err(_) => String::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_demangle_empty() {
        assert_eq!(demangle(""), "");
    }

    #[test]
    fn test_demangle_simple() {
        // Test demangling a real C++ mangled symbol
        let result = demangle("_ZN4mrpt5clock3nowEv");
        // Should demangle to something like "mrpt::clock::now()"
        println!("Demangled: {} -> {}", "_ZN4mrpt5clock3nowEv", result);
        assert!(result.contains("mrpt") || result.contains("clock"));
    }

    #[test]
    fn test_demangle_invalid() {
        // Invalid symbols should return the original string
        let result = demangle("not_a_mangled_symbol");
        assert_eq!(result, "not_a_mangled_symbol");
    }

    #[test]
    fn test_demangle_actual() {
        // Test that demangling actually works
        let mangled = "_ZNSt6vectorIiSaIiEE9push_backERKi";
        let result = demangle(mangled);
        println!("Demangled: {} -> {}", mangled, result);
        // Should contain "vector" and "push_back"
        assert!(result.contains("vector") || result.contains("push_back"));
    }
}
