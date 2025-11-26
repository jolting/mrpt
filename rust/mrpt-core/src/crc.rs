// Mobile Robot Programming Toolkit (MRPT)
// https://www.mrpt.org/
//
// Copyright (c) 2005-2024, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// Released under BSD License. See: https://www.mrpt.org/License

//! CRC (Cyclic Redundancy Check) computation
//!
//! This module provides CRC16 and CRC32 computation functions.

/// Default polynomial for CRC16
pub const CRC16_GEN_POL: u16 = 0x8005;

/// Default polynomial for CRC32
pub const CRC32_GEN_POL: u32 = 0x04C11DB7;

/// Compute CRC16 checksum
pub fn compute_crc16(data: &[u8], gen_pol: u16) -> u16 {
    let mut crc: u16 = 0;
    let mut ab_data = [0u8; 2];

    for &byte in data {
        ab_data[1] = ab_data[0];
        ab_data[0] = byte;

        if crc & 0x8000 != 0 {
            crc = (crc & 0x7fff) << 1;
            crc ^= gen_pol;
        } else {
            crc <<= 1;
        }
        crc ^= u16::from_le_bytes(ab_data);
    }
    crc
}

/// Compute CRC32 checksum
pub fn compute_crc32(data: &[u8], gen_pol: u32) -> u32 {
    let mut crc: u32 = 0;

    for &byte in data {
        let temp1 = (crc >> 8) & 0x00FFFFFF;
        let temp2 = crc32_value(((crc as u8) ^ byte) as i32, gen_pol);
        crc = temp1 ^ temp2;
    }
    crc
}

/// Helper function for CRC32 computation
fn crc32_value(i: i32, crc32_polynomial: u32) -> u32 {
    let mut crc = i as u32;
    for _ in 0..8 {
        if crc & 1 != 0 {
            crc = (crc >> 1) ^ crc32_polynomial;
        } else {
            crc >>= 1;
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16() {
        let data = b"Hello, World!";
        let crc = compute_crc16(data, CRC16_GEN_POL);
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_crc32() {
        let data = b"Hello, World!";
        let crc = compute_crc32(data, CRC32_GEN_POL);
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_crc16_empty() {
        let data = b"";
        let crc = compute_crc16(data, CRC16_GEN_POL);
        assert_eq!(crc, 0);
    }

    #[test]
    fn test_crc32_empty() {
        let data = b"";
        let crc = compute_crc32(data, CRC32_GEN_POL);
        assert_eq!(crc, 0);
    }
}
