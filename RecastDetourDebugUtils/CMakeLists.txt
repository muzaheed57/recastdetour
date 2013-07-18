CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(recastdetourdebugutils_SRCS 
	Source/DebugDraw.cpp
	Source/DetourDebugDraw.cpp
	Source/RecastDebugDraw.cpp
	Source/RecastDump.cpp
)

SET(recastdetourdebugutils_HDRS
	Include/DebugDraw.h
	Include/DetourDebugDraw.h
	Include/RecastDebugDraw.h
	Include/RecastDump.h
)

INCLUDE_DIRECTORIES(Include 
	../Detour/Include
	../DetourTileCache/Include
	../Recast/Include
)

ADD_LIBRARY(RecastDetourDebugUtils ${recastdetourdebugutils_SRCS} ${recastdetourdebugutils_HDRS})

INSTALL(FILES ${recastdetourdebugutils_HDRS}
    DESTINATION include/recastDetourDebugUtils
)

INSTALL(TARGETS RecastDetourDebugUtils EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION bin/${PLATFORM_STR}
    LIBRARY DESTINATION lib/${PLATFORM_STR}
    ARCHIVE DESTINATION lib/${PLATFORM_STR}
)