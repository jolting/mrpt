/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Rust implementation wrappers for byte reversal functionality
// This file provides C++ wrappers that call into the Rust implementation via FFI

#include "core-precomp.h"  // Precompiled headers
//
#include <mrpt/core/reverse_bytes.h>

// Include Rust FFI header
#include <mrpt_core_ffi.h>

#include <cstdlib>
#include <cstring>

void mrpt::reverseBytesInPlace(bool&)
{
  // Nothing to do.
}

void mrpt::reverseBytesInPlace(uint8_t& /*v_in_out*/)
{
  // Nothing to do.
}

void mrpt::reverseBytesInPlace(int8_t& /*v_in_out*/)
{
  // Nothing to do.
}

void mrpt::reverseBytesInPlace(uint16_t& v_in_out)
{
  // Call Rust implementation
  v_in_out = mrpt_reverse_bytes_u16(v_in_out);
}

void mrpt::reverseBytesInPlace(int16_t& v_in_out)
{
  // Call Rust implementation (cast through uint16_t)
  uint16_t temp = static_cast<uint16_t>(v_in_out);
  temp = mrpt_reverse_bytes_u16(temp);
  v_in_out = static_cast<int16_t>(temp);
}

void mrpt::reverseBytesInPlace(uint32_t& v_in_out)
{
  // Call Rust implementation
  v_in_out = mrpt_reverse_bytes_u32(v_in_out);
}

void mrpt::reverseBytesInPlace(int32_t& v_in_out)
{
  // Call Rust implementation (cast through uint32_t)
  uint32_t temp = static_cast<uint32_t>(v_in_out);
  temp = mrpt_reverse_bytes_u32(temp);
  v_in_out = static_cast<int32_t>(temp);
}

void mrpt::reverseBytesInPlace(uint64_t& v_in_out)
{
  // Call Rust implementation
  v_in_out = mrpt_reverse_bytes_u64(v_in_out);
}

void mrpt::reverseBytesInPlace(int64_t& v_in_out)
{
  // Call Rust implementation (cast through uint64_t)
  uint64_t temp = static_cast<uint64_t>(v_in_out);
  temp = mrpt_reverse_bytes_u64(temp);
  v_in_out = static_cast<int64_t>(temp);
}

void mrpt::reverseBytesInPlace(float& v_in_out)
{
  // Call Rust implementation
  v_in_out = mrpt_reverse_bytes_f32(v_in_out);
}

void mrpt::reverseBytesInPlace(double& v_in_out)
{
  // Call Rust implementation
  v_in_out = mrpt_reverse_bytes_f64(v_in_out);
}

void mrpt::reverseBytesInPlace(std::chrono::time_point<mrpt::Clock>& v_in_out)
{
  int64_t val = v_in_out.time_since_epoch().count();
  // Call Rust implementation
  uint64_t temp = static_cast<uint64_t>(val);
  temp = mrpt_reverse_bytes_u64(temp);
  val = static_cast<int64_t>(temp);
  v_in_out = std::chrono::time_point<mrpt::Clock>(mrpt::Clock::duration(val));
}
