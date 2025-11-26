// Mobile Robot Programming Toolkit (MRPT)
// https://www.mrpt.org/
//
// Copyright (c) 2005-2024, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// Released under BSD License. See: https://www.mrpt.org/License

//! CPU feature detection
//!
//! This module provides runtime detection of CPU features like SSE, AVX, etc.

use once_cell::sync::Lazy;
use std::sync::Mutex;

/// CPU features that can be detected
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CpuFeature {
    MMX,
    POPCNT,
    SSE,
    SSE2,
    SSE3,
    SSSE3,
    SSE4_1,
    SSE4_2,
    AVX,
    AVX2,
}

/// CPU feature detection results
#[derive(Debug, Clone)]
pub struct CpuInfo {
    features: [bool; 10],
}

impl CpuInfo {
    /// Create a new CPU info with detected features
    fn new() -> Self {
        let mut info = CpuInfo {
            features: [false; 10],
        };
        info.detect();
        info
    }

    /// Detect CPU features
    fn detect(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            if is_x86_feature_detected!("mmx") {
                self.features[CpuFeature::MMX as usize] = true;
            }
            if is_x86_feature_detected!("popcnt") {
                self.features[CpuFeature::POPCNT as usize] = true;
            }
            if is_x86_feature_detected!("sse") {
                self.features[CpuFeature::SSE as usize] = true;
            }
            if is_x86_feature_detected!("sse2") {
                self.features[CpuFeature::SSE2 as usize] = true;
            }
            if is_x86_feature_detected!("sse3") {
                self.features[CpuFeature::SSE3 as usize] = true;
            }
            if is_x86_feature_detected!("ssse3") {
                self.features[CpuFeature::SSSE3 as usize] = true;
            }
            if is_x86_feature_detected!("sse4.1") {
                self.features[CpuFeature::SSE4_1 as usize] = true;
            }
            if is_x86_feature_detected!("sse4.2") {
                self.features[CpuFeature::SSE4_2 as usize] = true;
            }
            if is_x86_feature_detected!("avx") {
                self.features[CpuFeature::AVX as usize] = true;
            }
            if is_x86_feature_detected!("avx2") {
                self.features[CpuFeature::AVX2 as usize] = true;
            }
        }
    }

    /// Check if a specific CPU feature is supported
    pub fn supports(&self, feature: CpuFeature) -> bool {
        self.features[feature as usize]
    }

    /// Get a string representation of supported features
    pub fn features_as_string(&self) -> String {
        let features = [
            ("MMX", CpuFeature::MMX),
            ("POPCNT", CpuFeature::POPCNT),
            ("SSE", CpuFeature::SSE),
            ("SSE2", CpuFeature::SSE2),
            ("SSE3", CpuFeature::SSE3),
            ("SSSE3", CpuFeature::SSSE3),
            ("SSE4_1", CpuFeature::SSE4_1),
            ("SSE4_2", CpuFeature::SSE4_2),
            ("AVX", CpuFeature::AVX),
            ("AVX2", CpuFeature::AVX2),
        ];

        features
            .iter()
            .map(|(name, feat)| format!("{}:{}", name, if self.supports(*feat) { 1 } else { 0 }))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

/// Global CPU info instance
static CPU_INFO: Lazy<Mutex<CpuInfo>> = Lazy::new(|| Mutex::new(CpuInfo::new()));

/// Get the global CPU info
pub fn get_cpu_info() -> CpuInfo {
    CPU_INFO.lock().unwrap().clone()
}

/// Check if a specific CPU feature is supported
pub fn supports(feature: CpuFeature) -> bool {
    get_cpu_info().supports(feature)
}

/// Get a string representation of supported CPU features
pub fn features_as_string() -> String {
    get_cpu_info().features_as_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cpu_detection() {
        let info = get_cpu_info();
        let features_str = info.features_as_string();
        println!("CPU features: {}", features_str);
        assert!(!features_str.is_empty());
    }

    #[test]
    fn test_supports() {
        // Most modern x86_64 CPUs support SSE2
        #[cfg(target_arch = "x86_64")]
        {
            let has_sse2 = supports(CpuFeature::SSE2);
            println!("SSE2 support: {}", has_sse2);
        }
    }
}
