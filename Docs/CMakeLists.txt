#
# Copyright (c) 2013 MASA Group recastdetour@masagroup.net
#
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
#    claim that you wrote the original software. If you use this software
#    in a product, an acknowledgment in the product documentation would be
#    appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
#    misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.
#

FIND_PACKAGE(Doxygen)
INCLUDE(CMakeDependentOption)

# Options
CMAKE_DEPENDENT_OPTION(BUILD_RECASTDETOUR_DOC "Generate Recast/Detour user documentation using Doxygen" 
	ON "DOXYGEN_FOUND"
	OFF)

SET(DOXYGEN_INPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/out)

# target setup
IF(BUILD_RECASTDETOUR_DOC)
	GET_FILENAME_COMPONENT(DOXYGEN_CONFIGURED_INPUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile ABSOLUTE)

	CONFIGURE_FILE(${DOXYGEN_INPUT_DIR}/Doxyfile ${DOXYGEN_CONFIGURED_INPUT} @ONLY)

    CONFIGURE_FILE(${DOXYGEN_INPUT_DIR}/customdoxygen.css ${DOXYGEN_OUTPUT_DIR}/html/doxygen.css COPYONLY)

	ADD_CUSTOM_TARGET(RecastDetourDoc ALL
					  COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
					  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_CONFIGURED_INPUT}
					  COMMAND ${CMAKE_COMMAND} -E echo "Done."
					  WORKING_DIRECTORY ${DOXYGEN_INPUT_DIR})
					  
	# setup install rules
	INSTALL(DIRECTORY ${DOXYGEN_OUTPUT_DIR}/html
			DESTINATION doc
			COMPONENT Release)
ENDIF()