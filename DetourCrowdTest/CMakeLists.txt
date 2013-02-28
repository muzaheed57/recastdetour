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

SET(
  detourcrowdtest_SRCS
  Source/Application.cpp
  Source/DetourCrowdTest.cpp
  Source/CrowdSample.cpp
  Source/NavMeshCreator.cpp
  Source/ChunkyTriMesh.cpp
  Source/FileList.cpp
  Source/InputGeom.cpp
  Source/MeshLoaderObj.cpp
  Source/PerfTimer.cpp
  )
  
SET(catch_INC Contrib/catch/include)

SET(simplejson_DIR Contrib/SimpleJSON)
SET(simplejson_SRCS 
    ${simplejson_DIR}/JSON.cpp
    ${simplejson_DIR}/JSONValue.cpp
)
SET(simplejson_HDRS
    ${simplejson_DIR}/JSON.h
    ${simplejson_DIR}/JSONValue.h
	)
  
INCLUDE_DIRECTORIES(
	Include
	Contrib/
    ../Detour/Include
    ../Recast/Include
    ../DetourCrowd/Include
    ../DetourTileCache/Include
	${catch_INC}
	${simplejson_DIR}
	)
	
SOURCE_GROUP(sources FILES ${detourcrowdtest_SRCS})
SOURCE_GROUP(externals\\catch FILES ${catch_INC} ${simplejson_SRCS} ${simplejson_HDRS})
SOURCE_GROUP(cmake FILES CMakeLists.txt)

ADD_EXECUTABLE(DetourCrowdTest ${detourcrowdtest_SRCS} ${simplejson_SRCS} ${simplejson_HDRS})

SET_PROPERTY(TARGET DetourCrowdTest PROPERTY DEBUG_POSTFIX -gd)
SET_PROPERTY(TARGET DetourCrowdTest PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${RecastNavigation_SOURCE_DIR}/DetourCrowdTest/Bin)
SET_PROPERTY(TARGET DetourCrowdTest PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${RecastNavigation_SOURCE_DIR}/DetourCrowdTest/Bin)

TARGET_LINK_LIBRARIES(
  DetourCrowdTest
  DetourCrowd
  Detour
  Recast
  )

ADD_TEST(
  NAME DetourCrowdTest 
  CONFIGURATIONS Debug Release 
  WORKING_DIRECTORY ${RecastNavigation_SOURCE_DIR}/DetourCrowdTest/Bin
  COMMAND DetourCrowdTest -t DetourCrowdTest/*)

IF(MSVC)
  # Enable some linker optimisations
  SET_PROPERTY(TARGET DetourCrowdTest PROPERTY LINK_FLAGS_RELEASE "/OPT:REF /OPT:ICF")
	SET_PROPERTY(TARGET DetourCrowdTest PROPERTY LINK_FLAGS_MINSIZEREL "/OPT:REF /OPT:ICF")
	SET_PROPERTY(TARGET DetourCrowdTest PROPERTY LINK_FLAGS_RELWITHDEBINFO "/OPT:REF /OPT:ICF")
ENDIF(MSVC)