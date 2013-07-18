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
CMAKE_DEPENDENT_OPTION(BUILD_RECAST_DEMO "Build Recast graphical demo application" 
    ON "OPENGL_FOUND;SDL_FOUND"
    OFF)

IF(BUILD_RECAST_DEMO)
	SET(recastdemo_SRCS 
		Source/ChunkyTriMesh.cpp
		Source/ConvexVolumeTool.cpp
		Source/CrowdTool.cpp
		Source/Filelist.cpp
		Source/imgui.cpp
		Source/imguiRenderGL.cpp
		Source/InputGeom.cpp
		Source/main.cpp
		Source/MeshLoaderObj.cpp
		Source/NavMeshPruneTool.cpp
		Source/NavMeshTesterTool.cpp
		Source/OffMeshConnectionTool.cpp
		Source/PerfTimer.cpp
		Source/Sample.cpp
		Source/Sample_Debug.cpp
		Source/Sample_SoloMesh.cpp
		Source/Sample_TileMesh.cpp
		Source/Sample_TempObstacles.cpp
		Source/SampleInterfaces.cpp
		Source/SlideShow.cpp
		Source/TestCase.cpp
		Source/ValueHistory.cpp
		Contrib/fastlz/fastlz.c
	)

	SET(recastdemo_HDRS
		Include/ChunkyTriMesh.h
		Include/ConvexVolumeTool.h
		Include/CrowdTool.h
		Include/Filelist.h
		Include/imgui.h
		Include/imguiRenderGL.h
		Include/InputGeom.h
		Include/MeshLoaderObj.h
		Include/NavMeshPruneTool.h
		Include/NavMeshTesterTool.h
		Include/OffMeshConnectionTool.h
		Include/PerfTimer.h
		Include/Sample.h
		Include/Sample_Debug.h
		Include/Sample_SoloMesh.h
		Include/Sample_TileMesh.h
		Include/Sample_TempObstacles.h
		Include/SampleInterfaces.h
		Include/SDLMain.h
		Include/SlideShow.h
		Include/TestCase.h
		Include/ValueHistory.h
	)

	IF(MSVC)
		# Look in the Contrib directory for SDL when building with MSVC
		SET(CMAKE_PREFIX_PATH Contrib/SDL)
	ENDIF()

	INCLUDE_DIRECTORIES(Include 
		Contrib
		Contrib/fastlz
		../RecastDetourDebugUtils/Include
		../Detour/Include
		../DetourCrowd/Include
		../DetourTileCache/Include
		../Recast/Include
		${SDL_INCLUDE_DIR}
	)

	IF(XCODE)
		ADD_EXECUTABLE(RecastDemo MACOSX_BUNDLE Source/SDLMain.m Include/SDLMain.h ${recastdemo_SRCS} ${recastdemo_HDRS})
	ELSE()
		ADD_EXECUTABLE(RecastDemo WIN32 ${recastdemo_SRCS} ${recastdemo_HDRS})
	ENDIF()

	SET_PROPERTY(TARGET RecastDemo PROPERTY DEBUG_POSTFIX -gd)
    SET(RECASTDEMO_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Bin)
	SET_PROPERTY(TARGET RecastDemo PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${RECASTDEMO_BIN_DIR})
	SET_PROPERTY(TARGET RecastDemo PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${RECASTDEMO_BIN_DIR})
    IF(MSVC)
		# Enable some linker optimisations
		SET_PROPERTY(TARGET RecastDemo PROPERTY LINK_FLAGS_RELEASE "/OPT:REF /OPT:ICF")
		SET_PROPERTY(TARGET RecastDemo PROPERTY LINK_FLAGS_MINSIZEREL "/OPT:REF /OPT:ICF")
		SET_PROPERTY(TARGET RecastDemo PROPERTY LINK_FLAGS_RELWITHDEBINFO "/OPT:REF /OPT:ICF")
	ENDIF()

	TARGET_LINK_LIBRARIES(RecastDemo RecastDetourDebugUtils Detour DetourCrowd DetourTileCache Recast ${SDL_LIBRARY} ${OPENGL_LIBRARIES})

	INSTALL(DIRECTORY Bin/Meshes Bin/Tests
		DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
	)

	IF(MSVC)
		# Retrieve the SDL dll directory
        GET_FILENAME_COMPONENT(SDL_LIB_DIR ${SDLMAIN_LIBRARY} PATH)
        INSTALL(
            FILES Bin/DroidSans.ttf ${SDL_LIB_DIR}/SDL.dll
            DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
        )
	else()
		INSTALL(
			FILES Bin/DroidSans.ttf
			DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
		)
	endif()

	INSTALL(TARGETS RecastDemo 
	    RUNTIME DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
	    BUNDLE DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/bin/${PLATFORM_STR}
	    LIBRARY DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
	    ARCHIVE DESTINATION ${RECASTDETOUR_INSTALL_PREFIX}/lib/${PLATFORM_STR}
	)
ENDIF()
