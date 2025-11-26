// Mobile Robot Programming Toolkit (MRPT)
// https://www.mrpt.org/
//
// Copyright (c) 2005-2024, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// Released under BSD License. See: https://www.mrpt.org/License

//! Base64 encoding and decoding
//!
//! This module provides Base64 encoding and decoding functions.

const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Encode data to Base64 string
pub fn encode_base64(input_data: &[u8]) -> String {
    let mut output = String::with_capacity((input_data.len() * 4 + 2) / 3);
    let mut char_count = 0;
    let mut bits = 0u32;
    let mut cols = 0;

    for &byte in input_data {
        bits += byte as u32;
        char_count += 1;

        if char_count == 3 {
            output.push(ALPHABET[(bits >> 18) as usize] as char);
            output.push(ALPHABET[((bits >> 12) & 0x3f) as usize] as char);
            output.push(ALPHABET[((bits >> 6) & 0x3f) as usize] as char);
            output.push(ALPHABET[(bits & 0x3f) as usize] as char);
            cols += 4;
            if cols == 72 {
                output.push('\n');
                cols = 0;
            }
            bits = 0;
            char_count = 0;
        } else {
            bits <<= 8;
        }
    }

    if char_count != 0 {
        bits <<= 16 - (8 * char_count);
        output.push(ALPHABET[(bits >> 18) as usize] as char);
        output.push(ALPHABET[((bits >> 12) & 0x3f) as usize] as char);

        if char_count == 1 {
            output.push('=');
            output.push('=');
        } else {
            output.push(ALPHABET[((bits >> 6) & 0x3f) as usize] as char);
            output.push('=');
        }
        if cols > 0 {
            output.push('\n');
        }
    }

    output
}

/// Decode Base64 string to data
pub fn decode_base64(input: &str) -> Result<Vec<u8>, String> {
    // Build decode table
    let mut decoder = [0u8; 256];
    let mut in_alphabet = [false; 256];
    
    for (i, &c) in ALPHABET.iter().enumerate() {
        in_alphabet[c as usize] = true;
        decoder[c as usize] = i as u8;
    }

    let mut output = Vec::new();
    let mut bits = 0u32;
    let mut char_count = 0;
    let mut padding = 0;

    for c in input.bytes() {
        if c == b'=' {
            padding += 1;
            continue;
        }
        if c == b'\n' || c == b'\r' {
            continue;
        }
        if padding > 0 {
            return Err("Invalid Base64 string: data after padding".to_string());
        }
        if !in_alphabet[c as usize] {
            return Err(format!("Invalid character in Base64 string: {}", c as char));
        }

        bits = (bits << 6) | (decoder[c as usize] as u32);
        char_count += 1;

        if char_count == 4 {
            output.push((bits >> 16) as u8);
            output.push((bits >> 8) as u8);
            output.push(bits as u8);
            bits = 0;
            char_count = 0;
        }
    }

    // Handle remaining bits with padding
    if char_count > 0 {
        bits <<= 6 * (4 - char_count);
        output.push((bits >> 16) as u8);
        if char_count >= 3 {
            output.push((bits >> 8) as u8);
        }
    }

    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode() {
        let data = b"Hello, World!";
        let encoded = encode_base64(data);
        let decoded = decode_base64(&encoded).unwrap();
        assert_eq!(data, &decoded[..]);
    }

    #[test]
    fn test_empty() {
        let data = b"";
        let encoded = encode_base64(data);
        let decoded = decode_base64(&encoded).unwrap();
        assert_eq!(data, &decoded[..]);
    }

    #[test]
    fn test_known_encoding() {
        let data = b"Man";
        let encoded = encode_base64(data);
        assert!(encoded.starts_with("TWFu"));
    }
}
