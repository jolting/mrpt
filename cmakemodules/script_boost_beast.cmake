# Check boost beast for web interface
# ===================================================

# disabled on start
SET(CMAKE_MRPT_HAS_BOOST_BEAST 0)

# Leave at the user's choice to disable the web interface:
OPTION(DISABLE_WEB "Disable the build of the WEB interface" "ON")
IF(DISABLE_WEB)
    SET(CMAKE_MRPT_HAS_BOOST_BEAST 0)
ENDIF(DISABLE_WEB)

IF(NOT DISABLE_WEB)
	# find packages quiet
	FIND_PACKAGE(Boost QUIET COMPONENTS system)
	IF (Boost_SYSTEM_FOUND)
		#eventually beast will be included in boost
		find_path(BEAST_INCLUDE boost/beast.hpp
			PATHS ${Boost_INCLUDE_DIRS})
		# If using a new enough version of boost, use that beast
		IF("${BEAST_INCLUDE}" STREQUAL "BEAST_INCLUDE-NOTFOUND")


			SET(CMAKE_MRPT_HAS_BOOST_BEAST 1)	
			# Until then download beast
			include(ExternalProject)
			ExternalProject_Add(BoostBeast
				GIT_REPOSITORY    "https://github.com/boostorg/beast.git"
				GIT_TAG           "v124"
				SOURCE_DIR        "${MRPT_BINARY_DIR}/otherlibs/beast"
				BUILD_COMMAND     ""
				INSTALL_COMMAND   ""
			)
			SET(CMAKE_MRPT_HAS_BOOST_BEAST 1)
			#Add boost beast to the boost INCLUDE_DIRS
			SET(BEAST_INCLUDE_DIR "${MRPT_BINARY_DIR}/otherlibs/beast/include")
		ELSE()
			SET(BEAST_INCLUDE_DIR "${Boost_INCLUDE_DIR}")
			#once beast is in boost, it will already be in the Boost_INCLUDE_DIRS
			SET(CMAKE_MRPT_HAS_BOOST_BEAST 1)	
		ENDIF()
	ENDIF()
ENDIF()
