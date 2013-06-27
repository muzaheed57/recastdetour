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

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

PROJECT(RecastDetour)

SET(RECASTDETOUR_VERSION_MAJOR 2)
SET(RECASTDETOUR_VERSION_MINOR 0)
SET(RECASTDETOUR_VERSION_PATCH 0)

IF(NOT CMAKE_BUILD_TYPE)
#	SET(CMAKE_BUILD_TYPE "Debug")
	SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(CMAKE_DEBUG_POSTFIX "-gd")

# Compute platform string
INCLUDE(PlatformStr.cmake)
COMPUTE_PLATFORM_STR(PLATFORM_STR)

IF(MSVC)
	OPTION(USE_MSVC_FAST_FLOATINGPOINT "Use MSVC /fp:fast option" ON)
	IF(USE_MSVC_FAST_FLOATINGPOINT)
		ADD_DEFINITIONS(/fp:fast)
	ENDIF(USE_MSVC_FAST_FLOATINGPOINT)
	ADD_DEFINITIONS(/D _CRT_SECURE_NO_WARNINGS)
ENDIF(MSVC)

INSTALL(
    FILES recastnavigation.LICENSE.txt
    DESTINATION license
    CONFIGURATIONS Release
)

ENABLE_TESTING()

ADD_SUBDIRECTORY(RecastDetourDebugUtils)
ADD_SUBDIRECTORY(Detour)
ADD_SUBDIRECTORY(DetourCrowd)
ADD_SUBDIRECTORY(DetourTileCache)
ADD_SUBDIRECTORY(Recast)
ADD_SUBDIRECTORY(RecastDemo)
ADD_SUBDIRECTORY(DetourCrowdDemo)
ADD_SUBDIRECTORY(DetourSceneCreator)
ADD_SUBDIRECTORY(DetourCrowdTest)
ADD_SUBDIRECTORY(Docs)

# install the platform string computation module
INSTALL(FILES PlatformStr.cmake DESTINATION build/cmake)

# Configure and install the cmake package configuration
CONFIGURE_FILE(RecastNavigationConfig.cmake.in "${PROJECT_BINARY_DIR}/RecastNavigationConfig.cmake" @ONLY)
INSTALL(FILES "${PROJECT_BINARY_DIR}/RecastNavigationConfig.cmake" DESTINATION build/cmake)
INSTALL(EXPORT "RecastNavigationTargets-${PLATFORM_STR}" DESTINATION build/cmake)

# CPack configuration, builds a zip package using the target 'package' in 
SET(CPACK_GENERATOR "ZIP")
SET(CPACK_ZIP_COMPONENT_INSTALL OFF)
SET(CPACK_PACKAGE_VERSION_MAJOR ${RECASTDETOUR_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${RECASTDETOUR_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${RECASTDETOUR_VERSION_PATCH})

INCLUDE(CPack)
