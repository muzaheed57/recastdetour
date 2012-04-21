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

SET(detourcrowddemo_SRCS
    Source/Application.cpp
    Source/ChunkyTriMesh.cpp
    Source/DebugInfo.cpp
    Source/Filelist.cpp
    Source/imgui.cpp
    Source/imguiRenderGL.cpp
    Source/InputGeom.cpp
    Source/main.cpp
    Source/NavMeshCreator.cpp
    Source/MeshLoaderObj.cpp
    Source/PerfTimer.cpp
    Source/CrowdSample.cpp
    Source/SampleInterfaces.cpp
    Source/ValueHistory.cpp
    Source/Visualization.cpp
)

SET(detourcrowddemo_HDRS
    Include/Application.h
    Include/ChunkyTriMesh.h
    Include/DebugInfo.h
    Include/Filelist.h
    Include/imgui.h
    Include/imguiRenderGL.h
    Include/InputGeom.h
    Include/MeshLoaderObj.h
    Include/NavMeshCreator.h
    Include/PerfTimer.h
    Include/CrowdSample.h
    Include/SampleInterfaces.h
    Include/SDLMain.h
    Include/StaticConfiguration.h
    Include/ValueHistory.h
    Include/Visualization.h
)

SET(fastlz_DIR Contrib/fastlz)
SET(fastlz_SRCS 
    ${fastlz_DIR}/fastlz.c
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

SET(sdl_DIR Contrib/SDL)

IF(XCODE)
    SET(sdlmain_HDRS 
        Include/SDLMain.h
    )

    SET(sdlmain_SRCS 
        Source/SDLMain.m
    )
ENDIF()

SOURCE_GROUP(sources FILES ${detourcrowddemo_HDRS} ${detourcrowddemo_SRCS})
IF(XCODE)
    SOURCE_GROUP(externals\\SDLMain FILES ${sdlmain_HDRS} ${sdlmain_SRCS})
ENDIF()
SOURCE_GROUP(externals\\fastlz FILES ${fastlz_SRCS})
SOURCE_GROUP(externals\\SimpleJSON FILES ${simplejson_SRCS} ${simplejson_HDRS})
SOURCE_GROUP(cmake FILES CMakeLists.txt)

IF(MSVC)
    # Look in the Contrib directory for SDL when building with MSVC
    SET(CMAKE_PREFIX_PATH ${sdl_DIR})
ENDIF(MSVC)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(SDL REQUIRED)

INCLUDE_DIRECTORIES(Include 
    Contrib
    ${fastlz_DIR}
    ${simplejson_DIR}
    ../DebugUtils/Include
    ../Detour/Include
    ../DetourCrowd/Include
    ../DetourTileCache/Include
    ../Recast/Include
    ${SDL_INCLUDE_DIR}
)

IF(XCODE)
    ADD_EXECUTABLE(DetourCrowdDemo MACOSX_BUNDLE ${detourcrowddemo_SRCS} ${detourcrowddemo_HDRS} ${sdlmain_HDRS} ${sdlmain_SRCS} ${fastlz_SRCS} ${simplejson_HDRS} ${simplejson_SRCS})
ELSE(XCODE)
    ADD_EXECUTABLE(DetourCrowdDemo WIN32 ${detourcrowddemo_SRCS} ${detourcrowddemo_HDRS} ${fastlz_SRCS} ${simplejson_HDRS} ${simplejson_SRCS})
ENDIF(XCODE)

TARGET_LINK_LIBRARIES(DetourCrowdDemo DebugUtils Detour DetourCrowd DetourTileCache Recast ${SDL_LIBRARY} ${SDLMAIN_LIBRARY} ${OPENGL_LIBRARIES})

IF(MSVC)
    # Enable some linker optimisations
    SET_TARGET_PROPERTIES(DetourCrowdDemo PROPERTIES LINK_FLAGS_RELEASE "/OPT:REF /OPT:ICF")
    SET_TARGET_PROPERTIES(DetourCrowdDemo PROPERTIES LINK_FLAGS_MINSIZEREL "/OPT:REF /OPT:ICF")
    SET_TARGET_PROPERTIES(DetourCrowdDemo PROPERTIES LINK_FLAGS_RELWITHDEBINFO "/OPT:REF /OPT:ICF")
ENDIF(MSVC)

SET(EXECUTABLE_OUTPUT_PATH ${RecastNavigation_SOURCE_DIR}/DetourCrowdDemo/Bin)

