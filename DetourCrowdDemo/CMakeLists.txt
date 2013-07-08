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

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(SDL REQUIRED)
INCLUDE(CMakeDependentOption)

# Options
CMAKE_DEPENDENT_OPTION(BUILD_DETOURCROWD_DEMO "Build DetourCrowd graphical demo applications" 
    ON "OPENGL_FOUND;SDL_FOUND"
    OFF)

IF(BUILD_DETOURCROWD_DEMO)
    SET(detourcrowddemo_SRCS
        Source/Application.cpp
        Source/DebugInfo.cpp
        Source/Filelist.cpp
        Source/imgui.cpp
        Source/imguiRenderGL.cpp
        Source/main.cpp
        Source/SampleInterfaces.cpp
        Source/ValueHistory.cpp
        Source/Visualization.cpp
    )

    SET(detourcrowddemo_HDRS
        Include/Application.h
        Include/DebugInfo.h
        Include/Filelist.h
        Include/imgui.h
        Include/imguiRenderGL.h
        Include/SampleInterfaces.h
        Include/SDLMain.h
        Include/ValueHistory.h
        Include/Visualization.h
    )

    SET(fastlz_DIR Contrib/fastlz)
    SET(fastlz_SRCS 
        ${fastlz_DIR}/fastlz.c
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
    SOURCE_GROUP(cmake FILES CMakeLists.txt)

    IF(MSVC)
        # Look in the Contrib directory for SDL when building with MSVC
        SET(CMAKE_PREFIX_PATH ${sdl_DIR})
    ENDIF()

    INCLUDE_DIRECTORIES(Include 
        ${fastlz_DIR}
        ${SDL_INCLUDE_DIR}
        ../Detour/Include
        ../DetourCrowd/Include
        ../DetourSceneCreator/Include
        ../DetourTileCache/Include
        ../Recast/Include
        ../RecastDetourDebugUtils/Include
        Contrib
    )

    IF(XCODE)
        ADD_EXECUTABLE(DetourCrowdDemo MACOSX_BUNDLE ${detourcrowddemo_SRCS} ${detourcrowddemo_HDRS} ${sdlmain_HDRS} ${sdlmain_SRCS} ${fastlz_SRCS})
    ELSE()
        ADD_EXECUTABLE(DetourCrowdDemo WIN32 ${detourcrowddemo_SRCS} ${detourcrowddemo_HDRS} ${fastlz_SRCS})
    ENDIF()

    SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY DEBUG_POSTFIX -gd)
    SET(DETOURCROWDDEMO_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Bin)
    SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${DETOURCROWDDEMO_BIN_DIR})
    SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${DETOURCROWDDEMO_BIN_DIR})
    IF(MSVC)
        # Enable some linker optimisations
        SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY LINK_FLAGS_RELEASE "/OPT:REF /OPT:ICF")
    	SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY LINK_FLAGS_MINSIZEREL "/OPT:REF /OPT:ICF")
    	SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY LINK_FLAGS_RELWITHDEBINFO "/OPT:REF /OPT:ICF")
    ENDIF()

    TARGET_LINK_LIBRARIES(DetourCrowdDemo RecastDetourDebugUtils Detour DetourCrowd DetourSceneCreator DetourTileCache Recast ${SDL_LIBRARY} ${SDLMAIN_LIBRARY} ${OPENGL_LIBRARIES})

    INSTALL(DIRECTORY Bin/Meshes Bin/Samples
        DESTINATION bin/${PLATFORM_STR}
    )

    IF(MSVC)
        INSTALL(
            FILES Bin/DroidSans.ttf Bin/SDL.dll
            DESTINATION bin/${PLATFORM_STR}
        )
    ELSE()
        INSTALL(
            FILES Bin/DroidSans.ttf
            DESTINATION bin/${PLATFORM_STR}
        )
    ENDIF()

    INSTALL(TARGETS DetourCrowdDemo 
        RUNTIME DESTINATION bin/${PLATFORM_STR}
        BUNDLE DESTINATION bin/${PLATFORM_STR}
        LIBRARY DESTINATION lib/${PLATFORM_STR}
        ARCHIVE DESTINATION lib/${PLATFORM_STR}
    )
ENDIF()
