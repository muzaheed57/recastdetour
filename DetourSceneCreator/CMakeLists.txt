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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(DetourSceneCreator_SRCS
    Source/DetourSceneCreator.cpp
    Source/ChunkyTriMesh.cpp
    Source/InputGeom.cpp
    Source/NavMeshCreator.cpp
    Source/MeshLoaderObj.cpp
    Source/CrowdSample.cpp
	Source/BuildContext.cpp
	Source/PerfTimer.cpp
)

SET(DetourSceneCreator_HDRS
    Include/DetourSceneCreator.h
    Include/ChunkyTriMesh.h
    Include/InputGeom.h
    Include/MeshLoaderObj.h
    Include/NavMeshCreator.h
    Include/CrowdSample.h
    Include/StaticConfiguration.h
	Include/BuildContext.h
	Include/PerfTimer.h
)

SET(simplejson_DIR Contrib/SimpleJSON)
SET(simplejson_SRCS 
    ${simplejson_DIR}/JSON.cpp
    ${simplejson_DIR}/JSONValue.cpp
)
SET(simplejson_HDRS
    ${simplejson_DIR}/JSON.h
    ${simplejson_DIR}/JSONValue.h
)

INCLUDE_DIRECTORIES(Include 
    ${simplejson_DIR}
    ../RecastDetourDebugUtils/Include
    ../Detour/Include
    ../DetourCrowd/Include
    ../DetourTileCache/Include
    ../Recast/Include
)

ADD_LIBRARY(DetourSceneCreator ${DetourSceneCreator_SRCS} ${DetourSceneCreator_HDRS} ${simplejson_SRCS} ${simplejson_HDRS})

INSTALL(FILES ${DetourSceneCreator_HDRS}
    DESTINATION include/DetourSceneCreator
)

INSTALL(TARGETS DetourSceneCreator EXPORT "RecastNavigationTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION bin/${PLATFORM_STR}
    LIBRARY DESTINATION lib/${PLATFORM_STR}
    ARCHIVE DESTINATION lib/${PLATFORM_STR}
)

TARGET_LINK_LIBRARIES(DetourSceneCreator Detour DetourCrowd)

if(WIN32)
    set(CMAKE_CXX_FLAGS "/EHsc /GR- /W4")
else(WIN32)
    set(CMAKE_CXX_FLAGS "-Wall -ansi -O3")
endif(WIN32)
