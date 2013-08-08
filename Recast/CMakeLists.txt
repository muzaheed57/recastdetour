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
IF(IOS)
    # workaround a bug forbidding to install the built library (cf. http://www.cmake.org/Bug/view.php?id=12506)
    SET_TARGET_PROPERTIES(Recast PROPERTIES 
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/Debug"
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/Release")
ENDIF()

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "Building binaries containing both 32 and 64 bits versions." 
  ON "APPLE;NOT IOS" 
  OFF)
IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
	SET_TARGET_PROPERTIES(Recast PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
ENDIF()

INSTALL(FILES ${recast_HDRS}
    DESTINATION include/recast
)

INSTALL(TARGETS Recast EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION bin/${PLATFORM_STR}
    LIBRARY DESTINATION lib/${PLATFORM_STR}
    ARCHIVE DESTINATION lib/${PLATFORM_STR}
)