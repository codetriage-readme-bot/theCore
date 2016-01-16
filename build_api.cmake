# Might be reviewed
cmake_minimum_required(VERSION 3.3)

# Set for latter use
set(CORE_DIR ${CMAKE_CURRENT_LIST_DIR})

# Registers a project
macro(register_project project_name)

	# This check is intentionally copied from core's listfile
	# Reason is that if build API used a proper platform must be set
	# _before_ compiler definitions and core inclusion.
	if (NOT DEFINED USE_PLATFORM)
		message(FATAL_ERROR "USE_PLATFORM must be set in order to use valid platform")
	else()
		message("Platform will be used: ${USE_PLATFORM}")
		set(PLATFORM_NAME ${USE_PLATFORM}) # For convinience
	endif()

	set(PROJECT_DIR ${CMAKE_CURRENT_DIR}/${project_name})
	set(PLATFORM_DIR ${CORE_DIR}/platform/${PLATFORM_NAME})

	# Export the platform name
	#set(PLATFORM_NAME ${platform_name})

	# Pick common compiler definitions for given platform
	include(${PLATFORM_DIR}/compiler/compiler.cmake)

	# Make sure the core is included
	add_subdirectory(${CORE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/core)

	# Make binary from the project object file
	add_custom_target(${PROJECT_NAME}.bin ALL
	COMMAND ${CMAKE_OBJCOPY} --output-format=binary
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME} ${PROJECT_NAME}.bin
	DEPENDS ${PROJECT_NAME}
	COMMENT "Making binary ${PROJECT_NAME}"
	)

	# Clean binary on 'make clean' call
	set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_NAME}.bin)
endmacro()

