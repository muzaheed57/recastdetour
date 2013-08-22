CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(detourtilecache_SRCS 
	Source/DetourTileCache.cpp
	Source/DetourTileCacheBuilder.cpp
)

SET(detourtilecache_HDRS
	Include/DetourTileCache.h
	Include/DetourTileCacheBuilder.h
)

INCLUDE_DIRECTORIES(Include 
	../Detour/Include
	../Recast/Include
)

ADD_LIBRARY(DetourTileCache ${detourtilecache_SRCS} ${detourtilecache_HDRS})
IF(IOS)
    # workaround a bug forbidding to install the built library (cf. http://www.cmake.org/Bug/view.php?id=12506)
    SET_TARGET_PROPERTIES(DetourTileCache PROPERTIES 
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/Debug"
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/Release")
ENDIF()

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "Building binaries containing both 32 and 64 bits versions." 
  ON "APPLE;NOT IOS" 
  OFF)
IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
    SET_TARGET_PROPERTIES(DetourTileCache PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
ENDIF()

INSTALL(FILES ${detourtilecache_HDRS}
    DESTINATION include/detourTileCache
)

INSTALL(TARGETS DetourTileCache EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION bin/${PLATFORM_STR}
    LIBRARY DESTINATION lib/${PLATFORM_STR}
    ARCHIVE DESTINATION lib/${PLATFORM_STR}
)
