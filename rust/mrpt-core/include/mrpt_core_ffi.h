/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Clock source enum - must match Rust CClockSource
typedef enum {
    MRPT_CLOCK_REALTIME = 0,
    MRPT_CLOCK_MONOTONIC = 1,
    MRPT_CLOCK_SIMULATED = 2
} mrpt_clock_source_t;

// Clock functions
int64_t mrpt_clock_now(void);
double mrpt_clock_now_double(void);
int64_t mrpt_clock_from_double(double t);
double mrpt_clock_to_double(int64_t t);
void mrpt_clock_set_active(mrpt_clock_source_t source);
mrpt_clock_source_t mrpt_clock_get_active(void);
void mrpt_clock_set_simulated_time(uint64_t time);
uint64_t mrpt_clock_get_simulated_time(void);
uint64_t mrpt_clock_reset_monotonic_epoch(void);

// Byte reversal functions
uint16_t mrpt_reverse_bytes_u16(uint16_t value);
uint32_t mrpt_reverse_bytes_u32(uint32_t value);
uint64_t mrpt_reverse_bytes_u64(uint64_t value);
float mrpt_reverse_bytes_f32(float value);
double mrpt_reverse_bytes_f64(double value);

#ifdef __cplusplus
}
#endif
