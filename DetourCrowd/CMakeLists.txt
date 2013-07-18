CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(detourcrowd_SRCS
	Source/DetourPathCorridor.cpp
	Source/DetourLocalBoundary.cpp
	Source/DetourPathQueue.cpp
	Source/DetourCrowd.cpp
	Source/DetourProximityGrid.cpp
	Source/DetourCollisionAvoidance.cpp
	Source/DetourPathFollowing.cpp
	Source/DetourFlockingBehavior.cpp
	Source/DetourSeparationBehavior.cpp
	Source/DetourCohesionBehavior.cpp
	Source/DetourAlignmentBehavior.cpp
	Source/DetourGoToBehavior.cpp
	Source/DetourSeekBehavior.cpp
	Source/DetourPipelineBehavior.cpp
	Source/DetourBehavior.cpp
)

SET(detourcrowd_HDRS
	Include/DetourPathCorridor.h
	Include/DetourCrowd.h
	Include/DetourLocalBoundary.h
	Include/DetourProximityGrid.h
	Include/DetourPathQueue.h
	Include/DetourCollisionAvoidance.h
	Include/DetourPathFollowing.h
	Include/DetourSeekBehavior.h
	Include/DetourFlockingBehavior.h
	Include/DetourSeparationBehavior.h
	Include/DetourCohesionBehavior.h
	Include/DetourAlignmentBehavior.h
	Include/DetourGoToBehavior.h
	Include/DetourSteeringBehavior.h
	Include/DetourBehavior.h
	Include/DetourPipelineBehavior.h
	Include/DetourParametrizedBehavior.h
)

INCLUDE_DIRECTORIES(Include 
	../Detour/Include
	../DetourTileCache
	../Recast/Include
)

ADD_LIBRARY(DetourCrowd ${detourcrowd_SRCS} ${detourcrowd_HDRS})

INSTALL(FILES ${detourcrowd_HDRS}
    DESTINATION include/detourCrowd
)

INSTALL(TARGETS DetourCrowd EXPORT "RecastDetourTargets-${PLATFORM_STR}"
    RUNTIME DESTINATION bin/${PLATFORM_STR}
    LIBRARY DESTINATION lib/${PLATFORM_STR}
    ARCHIVE DESTINATION lib/${PLATFORM_STR}
)

if(WIN32)
    set(CMAKE_CXX_FLAGS "/EHsc /GR- /W4")
else(WIN32)
    set(CMAKE_CXX_FLAGS "-Wall -ansi -O3")
endif(WIN32)
