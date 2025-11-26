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

#include <mrpt/core/demangle.h>

// Rust FFI declarations
extern "C" {
int mrpt_demangle(const char* symbol_name, char* buffer, size_t buffer_len);
}

std::string mrpt::demangle(const std::string& symbolName)
{
  if (symbolName.empty()) return {};

  // Try Rust implementation first
  std::string buffer;
  buffer.resize(2048);
  
  int result = mrpt_demangle(symbolName.c_str(), &buffer[0], buffer.size());
  
  if (result > 0) {
    buffer.resize(result);
    return buffer;
  }

  // Fallback: For Windows, we need platform-specific demangling
  // For now, the Rust implementation handles platform differences
  return symbolName;
}
