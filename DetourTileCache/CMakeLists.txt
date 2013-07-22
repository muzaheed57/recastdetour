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

IF (APPLE)
    IF (NOT DEFINED BUILD_RECASTDETOUR_FAT_LIBRARIES)
        OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "If selected, built libraries will contain both 32 and 64 bits versions." OFF)
    ENDIF()
    IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
    	SET_TARGET_PROPERTIES(DetourTileCache PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
    ENDIF()
ENDIF()

INSTALL(FILES ${detourtilecache_HDRS}
    DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/include/detourTileCache
)

INSTALL(TARGETS DetourTileCache EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
    LIBRARY DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
    ARCHIVE DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
)
