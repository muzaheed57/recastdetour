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

INSTALL(FILES ${detourtilecache_HDRS}
    DESTINATION include/detourTileCache
)

INSTALL(TARGETS DetourTileCache EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION bin/${PLATFORM_STR}
    LIBRARY DESTINATION lib/${PLATFORM_STR}
    ARCHIVE DESTINATION lib/${PLATFORM_STR}
)
