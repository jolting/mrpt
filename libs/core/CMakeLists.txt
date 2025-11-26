#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	core
	# Dependencies
	Threads  # invoke find_dependency(Threads) in "*-config.cmake"
	)

if(BUILD_mrpt-core)
	# Get current source list BEFORE any modifications
	get_target_property(CORE_SOURCES core SOURCES)
	
	# Always filter out any files in the rust/ subdirectory from auto-discovered sources
	# These should only be included when MRPT_USE_RUST_CORE is ON
	set(FILTERED_SOURCES "")
	foreach(src ${CORE_SOURCES})
		# Exclude anything in src/rust/ subdirectory
		if(NOT (src MATCHES "/rust/" OR src MATCHES "\\\\rust\\\\"))
			list(APPEND FILTERED_SOURCES ${src})
		endif()
	endforeach()
	
	# Check if we should use Rust implementation
	if(MRPT_USE_RUST_CORE AND TARGET mrpt_core_rust)
		message(STATUS "Using Rust implementation for mrpt-core")
		
		# List of C++ source files that have Rust wrapper replacements
		# These files will be excluded when MRPT_USE_RUST_CORE is ON
		set(CPP_FILES_WITH_RUST_REPLACEMENTS
			Clock.cpp
			reverse_bytes.cpp
		)
		
		# List of Rust wrapper files from rust/ subdirectory
		set(RUST_WRAPPER_FILES
			${CMAKE_CURRENT_SOURCE_DIR}/src/rust/Clock.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/src/rust/reverse_bytes.cpp
		)
		
		# Filter out C++ files that have Rust replacements
		set(FINAL_SOURCES "")
		foreach(src ${FILTERED_SOURCES})
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
		message(STATUS "  Adding Rust wrapper files: rust/Clock.cpp, rust/reverse_bytes.cpp")
		
		# Set the final source list
		set_target_properties(core PROPERTIES SOURCES "${FINAL_SOURCES}")
		
		# Link the Rust library into the core target
		# This must be done early so that Rust symbols can be exported
		target_link_libraries(core PUBLIC mrpt_core_rust)
		
		# Note: We keep WINDOWS_EXPORT_ALL_SYMBOLS ON (default) so Rust symbols are re-exported
		# from the core DLL
		
		# Make sure Rust library is built
		add_dependencies(all_mrpt_libs mrpt_core_rust_build)
		
		# Note: IMPORTED targets cannot be used with install(TARGETS)
		# Install the Rust library file directly
		get_target_property(RUST_LIB_LOCATION mrpt_core_rust IMPORTED_LOCATION)
		install(FILES ${RUST_LIB_LOCATION}
			DESTINATION ${CMAKE_INSTALL_LIBDIR}
		)
		
		# Install the FFI header
		install(FILES ${CMAKE_SOURCE_DIR}/rust/mrpt-core/include/mrpt_core_ffi.h
			DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mrpt/core
		)
		
		# Skip building C++ sources for core
		set(MRPT_SKIP_CORE_CPP_SOURCES TRUE)
	else()
		message(STATUS "Using C++ implementation for mrpt-core")
		# Just use the filtered sources (without Rust wrapper files)
		set_target_properties(core PROPERTIES SOURCES "${FILTERED_SOURCES}")
		set(MRPT_SKIP_CORE_CPP_SOURCES FALSE)
	endif()
	
	# Enforce C++17 in all dependent projects (regardless of Rust/C++):
	mrpt_lib_target_requires_cpp17(core)
	
	# Fix a MSVC binary-breaking compatibility in MSVC 2017 15.8:
	if (MSVC)
		target_compile_definitions(core PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE)
	endif()

	# Platform-specific libraries needed even when using Rust (for backtrace/demangle):
	if(WIN32)
		target_link_libraries(core PRIVATE Dbghelp) # For mrpt::demangle(), mrpt::callStackBackTrace()
	else()
		target_link_libraries(core PRIVATE ${CMAKE_DL_LIBS}) # For mrpt::callStackBackTrace()
	endif()
	
	# Includes: <mrpt/config.h> & <mrpt/version.h> config headers:
	# This is needed for both C++ and Rust implementations
	target_include_directories(core BEFORE PUBLIC
		$<BUILD_INTERFACE:${MRPT_CONFIG_FILE_INCLUDE_DIR}>
		$<INSTALL_INTERFACE:include>
	)
	
	# Only configure C++ core if not using Rust
	if(NOT MRPT_SKIP_CORE_CPP_SOURCES)

	target_link_libraries(core PRIVATE Threads::Threads) # for WorkerThreads

	if (CMAKE_MRPT_HAS_BFD)
		target_link_libraries(core PRIVATE ${BFD_LIBRARIES})
	endif()
	
	endif() # NOT MRPT_SKIP_CORE_CPP_SOURCES

	# These compile definitions are needed even when using Rust:
	target_compile_definitions(core PRIVATE MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH=${MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH})
	if (MRPT_EXCEPTIONS_WITH_CALL_STACK)
		target_compile_definitions(core PRIVATE MRPT_EXCEPTIONS_WITH_CALL_STACK)
	endif()

endif() # BUILD_mrpt-core
