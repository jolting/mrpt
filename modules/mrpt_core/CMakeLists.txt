# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_core LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)

# find dependencies:
find_package(Threads REQUIRED)

include(./cmake/bfd.cmake REQUIRED)
include(./cmake/detect_functions_headers.cmake REQUIRED)
include(./cmake/detect_word_size.cmake REQUIRED)
include(./cmake/exceptions_and_backtraces.cmake REQUIRED)
include(./cmake/max_alignment.cmake REQUIRED)
include(./cmake/simd.cmake REQUIRED)
include(./cmake/tbb.cmake REQUIRED)

include(./cmake/script_create_version_h.cmake REQUIRED)

set(MRPT_WITH_MATLAB_WRAPPER OFF CACHE BOOL "Build with compatibility options for MEX wrapper?.")
include(./cmake/script_matlab.cmake REQUIRED)      # Support for Matlab MEX functions generation
#if(MRPT_WITH_MATLAB_WRAPPER)
#    add_subdirectory(mex/apps)# The MEX applications
#endif()


# Option to use Rust implementation
option(MRPT_USE_RUST_CORE "Use Rust implementation for core functions" OFF)

# Build Rust library if enabled
if(MRPT_USE_RUST_CORE)
  # Add Rust library build target
  # In colcon builds, CMAKE_SOURCE_DIR is the module directory, so go up two levels
  set(RUST_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../rust")
  if(NOT EXISTS "${RUST_ROOT_DIR}")
    # Fallback for standalone builds
    set(RUST_ROOT_DIR "${CMAKE_SOURCE_DIR}/rust")
  endif()
  add_subdirectory(${RUST_ROOT_DIR} ${CMAKE_BINARY_DIR}/rust)
endif()

# define lib:
set(LIB_SRCS
  src/constexpr_for_unittest.cpp
  src/cpu_unittest.cpp
  src/bits_math_unittest.cpp
  src/reverse_bytes_unittest.cpp
  src/Clock.cpp
  src/format_unittest.cpp
  src/demangle.cpp
  src/format.cpp
  src/exceptions.cpp
  src/WorkerThreadsPool_unittest.cpp
  src/cpu.cpp
  src/exception_unittest.cpp
  src/reverse_bytes.cpp
  src/backtrace.cpp
  src/aligned_malloc_unittest.cpp
  src/Clock_unittest.cpp
  src/lock_helper_unittest.cpp
  src/from_string_unittest.cpp
  src/aligned_malloc.cpp
  src/WorkerThreadsPool.cpp
  src/is_defined_unittest.cpp
  src/round_unittest.cpp
  src/winerror2str.cpp
)

# Filter out rust/ subdirectory files and apply Rust replacements if enabled
if(MRPT_USE_RUST_CORE AND TARGET mrpt_core_rust)
  message(STATUS "Using Rust implementation for mrpt-core")
  
  # List of C++ source files that have Rust wrapper replacements
  set(CPP_FILES_WITH_RUST_REPLACEMENTS
    Clock.cpp
    reverse_bytes.cpp
    format.cpp
    exceptions.cpp
    cpu.cpp
    aligned_malloc.cpp
    demangle.cpp
  )
  
  # List of Rust wrapper files from rust/ subdirectory
  set(RUST_WRAPPER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rust/Clock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rust/reverse_bytes.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rust/format.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rust/exceptions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rust/cpu.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rust/aligned_malloc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rust/demangle.cpp
  )
  
  # Remove C++ files that have Rust replacements
  set(FINAL_SOURCES "")
  foreach(src ${LIB_SRCS})
    set(KEEP_FILE TRUE)
    foreach(cpp_file ${CPP_FILES_WITH_RUST_REPLACEMENTS})
      if(src MATCHES "/${cpp_file}$" OR src MATCHES "\\\\${cpp_file}$")
        set(KEEP_FILE FALSE)
        message(STATUS "  Excluding C++ file (using Rust wrapper): ${cpp_file}")
        break()
      endif()
    endforeach()
    if(KEEP_FILE)
      list(APPEND FINAL_SOURCES ${src})
    endif()
  endforeach()
  
  # Add Rust wrapper files
  list(APPEND FINAL_SOURCES ${RUST_WRAPPER_FILES})
  message(STATUS "  Adding Rust wrapper files from rust/ directory:")
  foreach(rust_file ${RUST_WRAPPER_FILES})
    get_filename_component(rust_filename ${rust_file} NAME)
    message(STATUS "    - rust/${rust_filename}")
  endforeach()
  
  set(LIB_SRCS ${FINAL_SOURCES})
endif()

set(LIB_PUBLIC_HDRS
  include/mrpt/3rdparty/spimpl.h
  include/mrpt/registerAllClasses.h
  include/mrpt/core/optional_ref.h
  include/mrpt/core/backtrace.h
  include/mrpt/core/alignment_req.h
  include/mrpt/core/aligned_std_vector.h
  include/mrpt/core/round.h
  include/mrpt/core/cpu.h
  include/mrpt/core/WorkerThreadsPool.h
  include/mrpt/core/Clock.h
  include/mrpt/core/bits_mem.h
  include/mrpt/core/lock_helper.h
  include/mrpt/core/SSE_types.h
  include/mrpt/core/format.h
  include/mrpt/core/safe_pointers.h
  include/mrpt/core/bits_math.h
  include/mrpt/core/exceptions.h
  include/mrpt/core/constexpr_for.h
  include/mrpt/core/reverse_bytes.h
  include/mrpt/core/from_string.h
  include/mrpt/core/aligned_allocator.h
  include/mrpt/core/is_defined.h
  include/mrpt/core/abs_diff.h
  include/mrpt/core/winerror2str.h
  include/mrpt/core/demangle.h
  include/mrpt/core/is_shared_ptr.h
  include/mrpt/core/aligned_std_basicstring.h
  include/mrpt/core/integer_select.h
  include/mrpt/core/bit_cast.h
  include/mrpt/core/pimpl.h
  include/mrpt/core/get_env.h
  include/mrpt/core/Stringifyable.h
  include/mrpt/core/common.h
  include/mrpt/core/initializer.h
  include/mrpt/core/SSE_macros.h
  include/mrpt/core/byte_manip.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
#  PUBLIC_LINK_LIBRARIES
#   xxx
#  PRIVATE_LINK_LIBRARIES
#  CMAKE_DEPENDENCIES
#   xxx
)

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads) # for WorkerThreads

# Link Rust library if enabled
if(MRPT_USE_RUST_CORE AND TARGET mrpt_core_rust)
  target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_core_rust)
  # Include directory is already set as INTERFACE_INCLUDE_DIRECTORIES on mrpt_core_rust
  message(STATUS "Linked Rust library to mrpt-core")
endif()

# Fix a MSVC binary-breaking compatibility in MSVC 2017 15.8:
if (MSVC)
  target_compile_definitions(${PROJECT_NAME} PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()

if(WIN32)
  target_link_libraries(${PROJECT_NAME} PRIVATE Dbghelp) # For mrpt::demangle(), mrpt::callStackBackTrace()
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS}) # For mrpt::callStackBackTrace()
endif()

if (CMAKE_MRPT_HAS_BFD)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${BFD_LIBRARIES})
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH=${MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH}U)
if (MRPT_EXCEPTIONS_WITH_CALL_STACK)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_EXCEPTIONS_WITH_CALL_STACK)
endif()

# Python wrapper:
mrpt_add_python_module(core
  python_bindings/${PROJECT_NAME}_py.cpp
)

