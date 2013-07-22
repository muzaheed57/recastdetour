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

IF (APPLE)
    IF (NOT DEFINED BUILD_RECASTDETOUR_FAT_LIBRARIES)
	    OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "If selected, built libraries will contain both 32 and 64 bits versions." OFF)
    ENDIF()
    IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
    	SET_TARGET_PROPERTIES(RecastDetourDebugUtils PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
    ENDIF()
ENDIF()

INSTALL(FILES ${recastdetourdebugutils_HDRS}
    DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/include/recastDetourDebugUtils
)

INSTALL(TARGETS RecastDetourDebugUtils EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
    LIBRARY DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
    ARCHIVE DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
)