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

#include <mrpt/core/aligned_allocator.h>

// Rust FFI declarations
extern "C" {
void* mrpt_aligned_malloc(size_t size, size_t alignment);
void* mrpt_aligned_calloc(size_t bytes, size_t alignment);
void mrpt_aligned_free(void* ptr, size_t size, size_t alignment);
}

// Global storage for tracking allocations (needed for free)
// This is a simple approach - in production you might use a better solution
#include <unordered_map>
#include <mutex>

namespace {
struct AllocInfo {
  size_t size;
  size_t alignment;
};
std::unordered_map<void*, AllocInfo> g_alloc_map;
std::mutex g_alloc_mutex;
}

void* mrpt::aligned_calloc(size_t bytes, size_t alignment)
{
  void* ptr = mrpt_aligned_calloc(bytes, alignment);
  if (ptr) {
    std::lock_guard<std::mutex> lock(g_alloc_mutex);
    g_alloc_map[ptr] = {bytes, alignment};
  }
  return ptr;
}

void* mrpt::aligned_malloc(size_t size, size_t alignment)
{
  void* ptr = mrpt_aligned_malloc(size, alignment);
  if (ptr) {
    std::lock_guard<std::mutex> lock(g_alloc_mutex);
    g_alloc_map[ptr] = {size, alignment};
  }
  return ptr;
}

void mrpt::aligned_free(void* ptr)
{
  if (!ptr) return;
  
  AllocInfo info;
  {
    std::lock_guard<std::mutex> lock(g_alloc_mutex);
    auto it = g_alloc_map.find(ptr);
    if (it != g_alloc_map.end()) {
      info = it->second;
      g_alloc_map.erase(it);
    } else {
      // Fallback - this shouldn't happen in normal usage
      info = {0, 16}; // Default alignment
    }
  }
  
  mrpt_aligned_free(static_cast<uint8_t*>(ptr), info.size, info.alignment);
}
