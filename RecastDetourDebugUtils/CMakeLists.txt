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
IF(IOS)
    # workaround a bug forbidding to install the built library (cf. http://www.cmake.org/Bug/view.php?id=12506)
    SET_TARGET_PROPERTIES(RecastDetourDebugUtils PROPERTIES 
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/Debug"
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/Release")
ENDIF()

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "Building binaries containing both 32 and 64 bits versions." 
  ON "APPLE;NOT IOS" 
  OFF)
IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
    SET_TARGET_PROPERTIES(RecastDetourDebugUtils PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
ENDIF()

INSTALL(FILES ${recastdetourdebugutils_HDRS}
    DESTINATION include/recastDetourDebugUtils
)

INSTALL(TARGETS RecastDetourDebugUtils EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION bin/${PLATFORM_STR}
    LIBRARY DESTINATION lib/${PLATFORM_STR}
    ARCHIVE DESTINATION lib/${PLATFORM_STR}
)