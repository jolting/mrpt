/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/core/cpu.h>
#include <mrpt/core/format.h>

// Rust FFI declarations
extern "C" {
bool mrpt_cpu_supports(uint32_t feature);
int mrpt_cpu_features_as_string(char* buffer, size_t buffer_len);
}

namespace mrpt::cpu::internal
{
CPU_analyzer& CPU_analyzer::Instance() noexcept
{
  static CPU_analyzer o;
  return o;
}

void CPU_analyzer::detect_impl() noexcept
{
  // Call Rust implementation for feature detection
  using namespace mrpt::cpu;
  
  feat(feature::MMX) = mrpt_cpu_supports(0);
  feat(feature::POPCNT) = mrpt_cpu_supports(1);
  feat(feature::SSE) = mrpt_cpu_supports(2);
  feat(feature::SSE2) = mrpt_cpu_supports(3);
  feat(feature::SSE3) = mrpt_cpu_supports(4);
  feat(feature::SSSE3) = mrpt_cpu_supports(5);
  feat(feature::SSE4_1) = mrpt_cpu_supports(6);
  feat(feature::SSE4_2) = mrpt_cpu_supports(7);
  feat(feature::AVX) = mrpt_cpu_supports(8);
  feat(feature::AVX2) = mrpt_cpu_supports(9);
}

}  // namespace mrpt::cpu::internal

std::string mrpt::cpu::features_as_string() noexcept
{
  char buffer[512];
  int result = mrpt_cpu_features_as_string(buffer, sizeof(buffer));
  
  if (result > 0) {
    return std::string(buffer);
  }
  
  // Fallback to manual construction
  const auto& feat = internal::CPU_analyzer::Instance();
  return mrpt::format(
      "MMX:%i POPCNT:%i SSE:%i SSE2:%i SSE3:%i SSSE3:%i SSE4_1:%i SSE4_2:%i "
      "AVX:%i AVX2:%i",
      feat.feat(feature::MMX) ? 1 : 0, feat.feat(feature::POPCNT) ? 1 : 0, 
      feat.feat(feature::SSE) ? 1 : 0, feat.feat(feature::SSE2) ? 1 : 0, 
      feat.feat(feature::SSE3) ? 1 : 0, feat.feat(feature::SSSE3) ? 1 : 0,
      feat.feat(feature::SSE4_1) ? 1 : 0, feat.feat(feature::SSE4_2) ? 1 : 0, 
      feat.feat(feature::AVX) ? 1 : 0, feat.feat(feature::AVX2) ? 1 : 0);
}
