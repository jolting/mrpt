#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	comms
	# Dependencies
	mrpt-base
	)

IF(CMAKE_MRPT_HAS_FTDI_SYSTEM)
	TARGET_LINK_LIBRARIES(mrpt-comms ${FTDI_LIBS})
ENDIF()

IF(CMAKE_MRPT_HAS_BOOST_BEAST)
	TARGET_INCLUDE_DIRECTORIES(mrpt-comms PRIVATE ${Boost_INCLUDE_DIR} ${BEAST_INCLUDE_DIR})
	TARGET_LINK_LIBRARIES(mrpt-comms ${Boost_SYSTEM_LIBRARY})
ENDIF()
