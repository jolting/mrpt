/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/format.h>

#include <cstdarg>

// This is the Rust-wrapper version that calls into Rust implementation
// The actual formatting is still done in C++ using vsnprintf since
// va_list cannot be safely passed across FFI boundary.
// 
// The Rust implementation provides helper utilities, but the core
// formatting remains in C++ for compatibility and safety.

std::string mrpt::format(const char* fmt, ...)
{
  if (!fmt) return {};

  int result = -1, length = 2048;
  std::string buffer;
  while (result == -1)
  {
    buffer.resize(length);

    va_list args;  // This must be done WITHIN the loop
    va_start(args, fmt);
#if defined(_MSC_VER)
    result = ::vsnprintf_s(&buffer[0], length, _TRUNCATE, fmt, args);
#else
    result = ::vsnprintf(&buffer[0], length, fmt, args);
#endif
    va_end(args);

    // Truncated?
    if (result >= length) result = -1;
    length *= 2;

    // Ok?
    if (result >= 0)
    {
      buffer.resize(result);
    }
  }
  return buffer;
}
