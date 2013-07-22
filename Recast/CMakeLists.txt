CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(recast_SRCS
	Source/Recast.cpp
	Source/RecastArea.cpp
	Source/RecastAlloc.cpp
	Source/RecastContour.cpp
	Source/RecastFilter.cpp
	Source/RecastLayers.cpp
	Source/RecastMesh.cpp
	Source/RecastMeshDetail.cpp
	Source/RecastRasterization.cpp
	Source/RecastRegion.cpp
)

SET(recast_HDRS
	Include/Recast.h
	Include/RecastAlloc.h
	Include/RecastAssert.h
)

INCLUDE_DIRECTORIES(Include)

ADD_LIBRARY(Recast ${recast_SRCS} ${recast_HDRS})

IF (APPLE)
    IF (NOT DEFINED BUILD_RECASTDETOUR_FAT_LIBRARIES)
	    OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "If selected, built libraries will contain both 32 and 64 bits versions." OFF)
    ENDIF()
    IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
    	SET_TARGET_PROPERTIES(Recast PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
    ENDIF()
ENDIF()

INSTALL(FILES ${recast_HDRS}
    DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/include/recast
)

INSTALL(TARGETS Recast EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
    LIBRARY DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
    ARCHIVE DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
)