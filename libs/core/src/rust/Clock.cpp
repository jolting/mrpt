/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Rust implementation wrappers for Clock functionality
// This file provides C++ wrappers that call into the Rust implementation via FFI

#include "core-precomp.h"  // Precompiled headers
//
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>

// Include Rust FFI header
#include <mrpt_core_ffi.h>

// Helper functions to convert between C++ and Rust clock sources
namespace
{
inline mrpt_clock_source_t toRustSource(mrpt::Clock::Source s)
{
  switch (s)
  {
    case mrpt::Clock::Source::Realtime:
      return MRPT_CLOCK_REALTIME;
    case mrpt::Clock::Source::Monotonic:
      return MRPT_CLOCK_MONOTONIC;
    case mrpt::Clock::Source::Simulated:
      return MRPT_CLOCK_SIMULATED;
    default:
      return MRPT_CLOCK_REALTIME;
  }
}

inline mrpt::Clock::Source fromRustSource(mrpt_clock_source_t s)
{
  switch (s)
  {
    case MRPT_CLOCK_REALTIME:
      return mrpt::Clock::Source::Realtime;
    case MRPT_CLOCK_MONOTONIC:
      return mrpt::Clock::Source::Monotonic;
    case MRPT_CLOCK_SIMULATED:
      return mrpt::Clock::Source::Simulated;
    default:
      return mrpt::Clock::Source::Realtime;
  }
}
}  // namespace

mrpt::Clock::time_point mrpt::Clock::now() noexcept
{
  // Call Rust implementation
  const int64_t timestamp = mrpt_clock_now();
  return time_point(duration(timestamp));
}

mrpt::Clock::time_point mrpt::Clock::fromDouble(const double t) noexcept
{
  // Call Rust implementation
  const int64_t timestamp = mrpt_clock_from_double(t);
  return time_point(duration(timestamp));
}

// Convert to time_t UNIX timestamp, with fractional part.
double mrpt::Clock::toDouble(const mrpt::Clock::time_point t) noexcept
{
  // Call Rust implementation
  return mrpt_clock_to_double(t.time_since_epoch().count());
}

void mrpt::Clock::setActiveClock(const Source s)
{
  ASSERT_(
      s == mrpt::Clock::Source::Realtime || s == mrpt::Clock::Source::Monotonic ||
      s == mrpt::Clock::Source::Simulated);

  // Call Rust implementation
  mrpt_clock_set_active(toRustSource(s));
}

mrpt::Clock::Source mrpt::Clock::getActiveClock()
{
  // Call Rust implementation
  return fromRustSource(mrpt_clock_get_active());
}

int64_t mrpt::Clock::resetMonotonicToRealTimeEpoch() noexcept
{
  // Call Rust implementation
  // Note: Rust implementation returns the offset, not the error
  // For compatibility, we return 0 (no error tracking in Rust version)
  mrpt_clock_reset_monotonic_epoch();
  return 0;
}

uint64_t mrpt::Clock::getMonotonicToRealtimeOffset()
{
  // Call Rust implementation
  return mrpt_clock_reset_monotonic_epoch();
}

void mrpt::Clock::setSimulatedTime(const time_point& t)
{
  // Call Rust implementation
  mrpt_clock_set_simulated_time(t.time_since_epoch().count());
}
