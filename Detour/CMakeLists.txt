CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(detour_SRCS
	Source/DetourAlloc.cpp
	Source/DetourCommon.cpp
	Source/DetourNavMesh.cpp
	Source/DetourNavMeshBuilder.cpp
	Source/DetourNavMeshQuery.cpp
	Source/DetourNode.cpp
)

SET(detour_HDRS
	Include/DetourAlloc.h
	Include/DetourAssert.h
	Include/DetourCommon.h
	Include/DetourNavMesh.h
	Include/DetourNavMeshBuilder.h
	Include/DetourNavMeshQuery.h
	Include/DetourNode.h
    Include/DetourStatus.h
)

INCLUDE_DIRECTORIES(Include)

ADD_LIBRARY(Detour ${detour_SRCS} ${detour_HDRS})

IF (APPLE)
	IF (NOT DEFINED BUILD_RECASTDETOUR_FAT_LIBRARIES)
	    OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "If selected, built libraries will contain both 32 and 64 bits versions." OFF)
    ENDIF()
    IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
    	SET_TARGET_PROPERTIES(Detour PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
    ENDIF()
ENDIF()

INSTALL(FILES ${detour_HDRS}
    DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/include/detour
)

INSTALL(TARGETS Detour EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
    LIBRARY DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
    ARCHIVE DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
)