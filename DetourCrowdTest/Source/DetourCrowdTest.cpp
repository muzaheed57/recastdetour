//
// Copyright (c) 2013 MASA Group recastdetour@masagroup.net
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "Application.h"
#include "CrowdSample.h"
#include "DetourCommon.h"
#include "InputGeom.h"

#define CATCH_CONFIG_MAIN // Generate automatically the main (one occurrence only)

#ifdef _MSC_VER
#pragma warning(push, 0)
#include <catch.hpp>
#pragma warning(pop)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#include <catch.hpp>
#pragma GCC diagnostic pop
#endif

#include <cstring>
#include <iostream>

/// Checks the equality of the two vectors on the xz plan.
bool vectorEqual2D(const float* v1, const float* v2)
{
	return v1[0] == v2[0] && v1[2] == v2[2];
}

TEST_CASE("DetourCrowdTest/UpdateAgentPosition", "We want to dynamically update the position of an agent with error checking")
{
	// Creation of a square
	float* vert = new float[12];
	int* tri = new int[6];

	vert[0] = 20.0; vert[1] = 0.0; vert[2] = 20.0;
	vert[3] = 20.0; vert[4] = 0.0; vert[5] = -20.0;
	vert[6] = -20.0; vert[7] = 0.0; vert[8] = -20.0;
	vert[9] = 20.0; vert[10] = 0.0; vert[11] = 20.0;

	tri[0] = 1; tri[1] = 2; tri[2] = 3;
	tri[3] = 3; tri[4] = 4; tri[5] = 1;

	// Creation of the simulation
	CrowdSample cs;
	InputGeom scene;
	dtNavMesh navMesh;

	REQUIRE(cs.initializeScene(&scene, vert, 4, tri, 2));
	REQUIRE(cs.initializeNavmesh(scene, &navMesh));

	// Creation of the crowd
	dtCrowd crowd;

	REQUIRE (crowd.init(20, 0.5, &navMesh));

	// Creation of the agents
	dtCrowdAgent ag1;
	dtCrowdAgentParams param1;
	memset(&param1, 0, sizeof(dtCrowdAgentParams));
	param1.radius = 0.2;
	param1.height = 1.7;
	param1.maxSpeed = 2.0;
	param1.maxAcceleration = 10.0;
	param1.collisionQueryRange = 4.0;
	param1.pathOptimizationRange = 6.0;
	param1.separationWeight = 1;
	param1.updateFlags = DT_CROWD_OBSTACLE_AVOIDANCE;
	param1.obstacleAvoidanceType = 0;

	float posAgt1[3] = {0, 0, 0};
	float destAgt1[3] = {15, 0, 0};

	// Adding the agent to the crowd
	int indexAgent1 = crowd.addAgent(posAgt1, &param1);

	float correctPosition[3] = {19, 0, 0};
	float wrongPosition[3] = {100, 0, 10};

	CHECK(crowd.updateAgentPosition(indexAgent1, correctPosition));
	// The agent has moved
	CHECK(vectorEqual2D(crowd.getAgent(indexAgent1)->npos, correctPosition));

	CHECK_FALSE(crowd.updateAgentPosition(indexAgent1, wrongPosition));
	// The agent has not moved
	CHECK(vectorEqual2D(crowd.getAgent(indexAgent1)->npos, correctPosition));
}
>>>>>>> 975aae2... Allowing users to dynamically change the position of an agent.

TEST_CASE("DetourCrowdTest/UpdateCrowd", "Test the different ways to update the agents inside a crowd")
{
	// Creation of a square
	float* vert = new float[12];
	int* tri = new int[6];

	vert[0] = 30.0; vert[1] = 0.0; vert[2] = 30.0;
	vert[3] = 30.0; vert[4] = 0.0; vert[5] = -30.0;
	vert[6] = -30.0; vert[7] = 0.0; vert[8] = -30.0;
	vert[9] = 30.0; vert[10] = 0.0; vert[11] = 30.0;

	tri[0] = 1; tri[1] = 2; tri[2] = 3;
	tri[3] = 3; tri[4] = 4; tri[5] = 1;

	// Creation of the navigation mesh
	CrowdSample cs;
	InputGeom scene;
	dtNavMesh navMesh;

	REQUIRE(cs.initializeScene(&scene, vert, 4, tri, 2));
	REQUIRE(cs.initializeNavmesh(scene, &navMesh));

	// Creation of the crowd
	dtCrowd crowd;

	REQUIRE(crowd.init(4, 0.5, &navMesh));

	// Creation of the agents
	dtCrowdAgentParams p1, p2;
	memset(&p1, 0, sizeof(dtCrowdAgentParams));
	p1.radius = 0.2;
	p1.height = 1.7;
	p1.maxSpeed = 2.0;
	p1.maxAcceleration = 10.0;
	p1.collisionQueryRange = 4.0;
	p1.pathOptimizationRange = 6.0;
	p1.separationWeight = 1;
	p1.updateFlags = DT_CROWD_OBSTACLE_AVOIDANCE;
	p1.obstacleAvoidanceType = 0;

	memcpy(&p2, &p1, sizeof(dtCrowdAgentParams));

	float a1ReferencePosition[3] = {-18, 0, -18};
	float a2ReferencePosition[3] = {18, 0, 18};
	float a3ReferencePosition[3] = {19, 0, -18};
	float a4ReferencePosition[3] = {18, 0, -18};
	float a1Destination[3] = {18, 0, 18};
	float a2Destination[3] = {-18, 0, -18};
	float a3Destination[3] = {1, 0, 0};
	float a4Destination[3] = {0, 0, 0};

	// Adding the agents to the crowd
	int a1Idx = crowd.addAgent(a1ReferencePosition, &p1);
	int a2Idx = crowd.addAgent(a2ReferencePosition, &p2);
    int indices[2] = {a1Idx, a2Idx};

	// Set the destination
    dtPolyRef a1TargetRef;
    dtPolyRef a2TargetRef;
	crowd.getNavMeshQuery()->findNearestPoly(a1Destination, crowd.getQueryExtents(), crowd.getFilter(), &a1TargetRef, a1Destination);
	crowd.getNavMeshQuery()->findNearestPoly(a2Destination, crowd.getQueryExtents(), crowd.getFilter(), &a2TargetRef, a2Destination);

	REQUIRE(crowd.requestMoveTarget(a1Idx, a1TargetRef, a1Destination));
	REQUIRE(crowd.requestMoveTarget(a2Idx, a2TargetRef, a2Destination));

	SECTION("Update all agents", "We update every agent at the time")
	{
		crowd.update(1.0, 0);

        CHECK(!dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));
		CHECK(!dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));
	}

	SECTION("Update specific agents", "We just want to update some specific agents")
	{
		// We just update the first agent
        dtVcopy(a1ReferencePosition,crowd.getAgent(a1Idx)->npos);
        dtVcopy(a2ReferencePosition,crowd.getAgent(a2Idx)->npos);

		crowd.update(1.0, 0, &a1Idx, 1);

        // a1Idx has moved.
        CHECK(!dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));

        // a2Idx hasn't
		CHECK(dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));

		// We just update the second agent
		dtVcopy(a1ReferencePosition,crowd.getAgent(a1Idx)->npos);

		crowd.update(1.0, 0, &a2Idx, 1);

        // a1Idx hasn't moved.
        CHECK(dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));

        // a2Idx has
		CHECK(!dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));
		
		// We update every agents using the index indices
		dtVcopy(a2ReferencePosition,crowd.getAgent(a2Idx)->npos);

		crowd.update(1.0, 0, indices, 2);
		
		// a1Idx has moved.
        CHECK(!dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));

        // a2Idx too
		CHECK(!dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));

		dtVcopy(a1ReferencePosition,crowd.getAgent(a1Idx)->npos);
		dtVcopy(a2ReferencePosition,crowd.getAgent(a2Idx)->npos);

		SECTION("Error cases", "Error cases when updating specific agents")
		{
			// Error case: the user gives a list of indexes that is too big
			// Size is 3, but there are only 2 agents
			int errorList[3] = {0, 1, 2};

			crowd.update(1.0, 0, errorList, 3);

			// a1Idx has moved.
			CHECK(!dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));

			// a2Idx too
			CHECK(!dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));

			// Error Case: the user gives invalid values in the list
			errorList[0] = 999; errorList[1] = -1;
			dtVcopy(a1ReferencePosition,crowd.getAgent(a1Idx)->npos);
			dtVcopy(a2ReferencePosition,crowd.getAgent(a2Idx)->npos);

			crowd.update(0.5, 0, errorList, 2);

			// a1Idx hasn't moved.
			CHECK(dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));

			// a2Idx neither
			CHECK(dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));
		}

		// Adding the agents to the crowd
		int a3Idx = crowd.addAgent(a3ReferencePosition, &p1);
		int a4Idx = crowd.addAgent(a4ReferencePosition, &p2);

		// Set the destinations
		dtPolyRef a3TargetRef;
		dtPolyRef a4TargetRef;
		crowd.getNavMeshQuery()->findNearestPoly(a3Destination, crowd.getQueryExtents(), crowd.getFilter(), &a3TargetRef, a3Destination);
		crowd.getNavMeshQuery()->findNearestPoly(a4Destination, crowd.getQueryExtents(), crowd.getFilter(), &a4TargetRef, a4Destination);

		REQUIRE(crowd.requestMoveTarget(a3Idx, a3TargetRef, a3Destination));
		REQUIRE(crowd.requestMoveTarget(a4Idx, a4TargetRef, a4Destination));

		REQUIRE(a4Idx != -1);
		REQUIRE(a3Idx != -1);

		dtVcopy(a1ReferencePosition, crowd.getAgent(a1Idx)->npos);
		dtVcopy(a2ReferencePosition, crowd.getAgent(a2Idx)->npos);
		dtVcopy(a3ReferencePosition, crowd.getAgent(a3Idx)->npos);
		dtVcopy(a4ReferencePosition, crowd.getAgent(a4Idx)->npos);

		crowd.removeAgent(a1Idx);
		crowd.update(0.5, 0);

		// Every agent has moved but the removed one
		CHECK(dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));
		CHECK(!dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));
		CHECK(!dtVequal(crowd.getAgent(a3Idx)->npos, a3ReferencePosition));
		CHECK(!dtVequal(crowd.getAgent(a4Idx)->npos, a4ReferencePosition));		
	}

	SECTION("Separate velocity and position", "We want to be able to update the velocity and the position of the agents separatly")
	{
		// Update only the environment
        dtVcopy(a1ReferencePosition,crowd.getAgent(a1Idx)->npos);
        dtVcopy(a2ReferencePosition,crowd.getAgent(a2Idx)->npos);

        float a1ReferenceVelocity[3], a2ReferenceVelocity[3];
        dtVcopy(a1ReferenceVelocity,crowd.getAgent(a1Idx)->vel);
        dtVcopy(a2ReferenceVelocity,crowd.getAgent(a2Idx)->vel);

		crowd.updateEnvironment(indices, 2);

        // positions and velocities should be the same.
        CHECK(dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));
        CHECK(dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));

        CHECK(dtVequal(crowd.getAgent(a1Idx)->vel, a1ReferenceVelocity));
        CHECK(dtVequal(crowd.getAgent(a2Idx)->vel, a2ReferenceVelocity));

		// Update velocity only
		crowd.updateVelocity(0.2f, 0, indices, 2);

        // positions should be the same.
        CHECK(dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));
        CHECK(dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));

        // velocities should have changed
        CHECK(!dtVequal(crowd.getAgent(a1Idx)->vel, a1ReferenceVelocity));
        CHECK(!dtVequal(crowd.getAgent(a2Idx)->vel, a2ReferenceVelocity));

		// Update position only
		dtVcopy(a1ReferenceVelocity, crowd.getAgent(a1Idx)->vel);
		dtVcopy(a2ReferenceVelocity, crowd.getAgent(a2Idx)->vel);

		crowd.updatePosition(0.2f, indices, 2);

		// positions should have changed.
        CHECK(!dtVequal(crowd.getAgent(a1Idx)->npos, a1ReferencePosition));
        CHECK(!dtVequal(crowd.getAgent(a2Idx)->npos, a2ReferencePosition));

        // velocities should be the same changed
        CHECK(dtVequal(crowd.getAgent(a1Idx)->vel, a1ReferenceVelocity));
        CHECK(dtVequal(crowd.getAgent(a2Idx)->vel, a2ReferenceVelocity));	
	}
}

TEST_CASE("DetourCrowdTest/InitCrowd", "Test whether the initialization of a crowd is successful")
{
	dtCrowd crowd;

	CHECK(crowd.getAgentCount() == 0);
}
