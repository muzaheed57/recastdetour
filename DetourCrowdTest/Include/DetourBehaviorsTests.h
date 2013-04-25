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

#ifndef DETOURBEHAVIORSTESTS_H
#define DETOURBEHAVIORSTESTS_H

#define CATCH_CONFIG_MAIN

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma warning(push, 0)
#include <catch.hpp>
#pragma warning(pop)
#pragma GCC diagnostic pop

#include "DetourCrowdTestUtils.h"

#include "DetourAlignmentBehavior.h"
#include "DetourCohesionBehavior.h"
#include "DetourCommon.h"
#include "DetourFlockingBehavior.h"
#include "DetourGoToBehavior.h"
#include "DetourSeekBehavior.h"
#include "DetourSeparationBehavior.h"

#include <cstring>


TEST_CASE("DetourCrowdTest/CustomBehavior", "Test whether the custom behaviors behave correctly")
{
	// Creation of a square
	float vert[12];
	int tri[6];

	vert[0] = 20.0; vert[1] = 0.0; vert[2] = 20.0;
	vert[3] = 20.0; vert[4] = 0.0; vert[5] = -20.0;
	vert[6] = -20.0; vert[7] = 0.0; vert[8] = -20.0;
	vert[9] = -20.0; vert[10] = 0.0; vert[11] = 20.0;

	tri[0] = 0; tri[1] = 1; tri[2] = 2;
	tri[3] = 2; tri[4] = 3; tri[5] = 0;

	dtCrowdAgentParams param1, param2;
	TestScene ts;
	dtCrowd* crowd = ts.createScene(param1, vert, tri);

	REQUIRE(crowd != 0);

	memcpy(&param2, &param1, sizeof(dtCrowdAgentParams));

	SECTION("Seek Behavior", "With the seeking behavior, an agent must move towards its target")
	{
		param1.steeringBehavior = new dtSeekBehavior;
		param1.seekTarget = crowd->getAgent(1);

		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {15, 0, 0};
		float destAgt2[] = {15, 0, 0};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1, &param1);
		int indexAgent2 = crowd->addAgent(posAgt2, &param2);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);

		dtPolyRef dest;
		// Set the destination
		crowd->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getQueryExtents(), crowd->getFilter(), &dest, 0);
		REQUIRE(crowd->requestMoveTarget(indexAgent2, dest, destAgt2));

		crowd->update(2.0, 0);

		float agt1NewPos[3];

		dtVcopy(agt1NewPos, crowd->getAgent(indexAgent1)->npos);	

		// The first agent should move to the right (since he is seeking the second agent)
		CHECK(posAgt1[0] < crowd->getAgent(indexAgent2)->npos[0]);

		delete param1.steeringBehavior;
		param1.steeringBehavior = 0;
	}

	SECTION("Flocking Behavior", "Unit tests about the flocking behavior")
	{
		dtCrowdAgentParams param3, param4, paramLeader;

		memcpy(&paramLeader, &param1, sizeof(dtCrowdAgentParams));
		param1.steeringBehavior = new dtFlockingBehavior(2, 1, 1);
		param2.steeringBehavior = param1.steeringBehavior;
		param1.flockingAgents = crowd->getAgents();
		param2.flockingAgents = crowd->getAgents();
		param1.flockingSeparationDistance = 2;
		param2.flockingSeparationDistance = 2;

		memcpy(&param3, &param1, sizeof(dtCrowdAgentParams));
		memcpy(&param4, &param1, sizeof(dtCrowdAgentParams));

		// Creation of the list of flocking neighbors for each agent
		param1.nbFlockingNeighbors = 4;
		param2.nbFlockingNeighbors = 4;
		param3.nbFlockingNeighbors = 4;
		param4.nbFlockingNeighbors = 4;

		int* toFlockWith1 = (int*) dtAlloc(sizeof(int) * param1.nbFlockingNeighbors, DT_ALLOC_TEMP);
		int* toFlockWith2 = (int*) dtAlloc(sizeof(int) * param2.nbFlockingNeighbors, DT_ALLOC_TEMP);
		int* toFlockWith3 = (int*) dtAlloc(sizeof(int) * param3.nbFlockingNeighbors, DT_ALLOC_TEMP);
		int* toFlockWith4 = (int*) dtAlloc(sizeof(int) * param4.nbFlockingNeighbors, DT_ALLOC_TEMP);

		toFlockWith1[0] = 1; toFlockWith1[1] = 2; toFlockWith1[2] = 3; toFlockWith1[3] = 4;
		toFlockWith2[0] = 0; toFlockWith2[1] = 2; toFlockWith2[2] = 3; toFlockWith2[3] = 4;
		toFlockWith3[0] = 0; toFlockWith3[1] = 1; toFlockWith3[2] = 3; toFlockWith3[3] = 4;
		toFlockWith4[0] = 0; toFlockWith4[1] = 1; toFlockWith4[2] = 2; toFlockWith4[3] = 4;

		param1.toFlockWith = toFlockWith1;
		param2.toFlockWith = toFlockWith2;
		param3.toFlockWith = toFlockWith3;
		param4.toFlockWith = toFlockWith4;

		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {1, 0, 1};
		float posAgt3[] = {1.5f, 0, 1};
		float posAgt4[] = {1, 0, 1.5f};
		float posLeader[] = {-0.5f, 0, -0.5f};
		float destLeader[] = {-14, 0, -14};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1, &param1);
		int indexAgent2 = crowd->addAgent(posAgt2, &param2);
		int indexAgent3 = crowd->addAgent(posAgt3, &param3);
		int indexAgent4 = crowd->addAgent(posAgt4, &param4);
		int indexLeader = crowd->addAgent(posLeader, &paramLeader);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);
		REQUIRE_FALSE(indexAgent3 == -1);
		REQUIRE_FALSE(indexAgent4 == -1);
		REQUIRE_FALSE(indexLeader == -1);

		// Set the destination
		crowd->getNavMeshQuery()->findNearestPoly(destLeader, crowd->getQueryExtents(), crowd->getFilter(), &crowd->getAgent(indexLeader)->targetRef, 0);
		REQUIRE(crowd->getAgent(indexLeader)->targetRef != 0);
		REQUIRE(crowd->requestMoveTarget(indexLeader, crowd->getAgent(indexLeader)->targetRef, destLeader));

		// We perform several update on the flocking group.
		for (int i = 0; i < 100; ++i)
			crowd->update(0.1f, 0);

		// Each agent should be heading toward the upper left corner
		for (int i = 0; i < 4; ++i)
		{
			float vel[3];
			dtVcopy(vel, crowd->getAgent(i)->vel);

			CHECK((vel[0] < 0 && vel[2] < 0));
		}

		// We perform several update in order to reach the point where the agents aren't moving anymore
		for (int i = 0; i < 1000; ++i)
			crowd->update(0.1f, 0);

		// We check that the minimal space between each agent is respected
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent2)->npos) > param1.flockingSeparationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent3)->npos) > param1.flockingSeparationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent4)->npos) > param1.flockingSeparationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent4)->npos, crowd->getAgent(indexAgent2)->npos) > param1.flockingSeparationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent4)->npos, crowd->getAgent(indexAgent3)->npos) > param1.flockingSeparationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent3)->npos, crowd->getAgent(indexAgent2)->npos) > param1.flockingSeparationDistance / 2.f);

		dtFree(toFlockWith1);
		dtFree(toFlockWith2);
		dtFree(toFlockWith3);
		dtFree(toFlockWith4);

		delete param1.steeringBehavior;
		param1.steeringBehavior = 0;
	}

	SECTION("Separation Behavior", "With the separation behavior, an agent tries to flee its target")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {2, 0, 0};
		float destAgt2[] = {2, 0, 0};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1, &param1);
		int indexAgent2 = crowd->addAgent(posAgt2, &param2);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);

		// Set the destination
		crowd->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getQueryExtents(), crowd->getFilter(), &crowd->getAgent(indexAgent2)->targetRef, 0);
		crowd->requestMoveTarget(indexAgent2, crowd->getAgent(indexAgent2)->targetRef, destAgt2);

		crowd->update(2.0, 0);

		float agt1NewVel[3];
		float nilVector[] = {0, 0, 0};

		dtVcopy(agt1NewVel, crowd->getAgent(indexAgent1)->vel);	

		// Since the distance is set to 0, the velocity of the agent should be nil
		CHECK(dtVequal(agt1NewVel, nilVector));

		// We now set the distance
		param1.collisionQueryRange = 5.f;
		crowd->updateAgentParameters(indexAgent1, &param1);

		crowd->update(2.0, 0);

		dtVcopy(agt1NewVel, crowd->getAgent(indexAgent1)->vel);	

		// However the agent has not moved yet since no target was specified
		CHECK(dtVequal(agt1NewVel, nilVector));

		// Set the behavior
		param1.steeringBehavior = new dtSeparationBehavior;
		param1.separationAgents = crowd->getAgents();
		param1.separationTargets = &indexAgent2;
		param1.separationNbTargets = 1;
		param1.separationWeight = 1.f;
		param1.separationDistance = 2.f;
		crowd->updateAgentParameters(indexAgent1, &param1);

		crowd->update(2.0, 0);

		// The agent should now be moving to the left (away from the target)
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent2)->npos) > 2.1);

		// We perform several updates
		for (int i = 0; i < 100; ++i)
			crowd->update(1.0, 0);

		// The agent should have moved to the left far enough to respect the minimal distance
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent2)->npos) >= param1.separationDistance);

		delete param1.steeringBehavior;
		param1.steeringBehavior = 0;
	}

	SECTION("Alignment Behavior", "With the alignment behavior, an agent will align its velocity to its target")
	{
		dtCrowdAgentParams param3;
		memcpy(&param3, &param2, sizeof(dtCrowdAgentParams));
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {0, 0, 1};
		float posAgt3[] = {0, 0, -1};
		float destAgt2[] = {15, 0, 1};
		float destAgt3[] = {15, 0, -1};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1, &param1);
		int indexAgent2 = crowd->addAgent(posAgt2, &param2);
		int indexAgent3 = crowd->addAgent(posAgt3, &param3);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);
		REQUIRE_FALSE(indexAgent3 == -1);

		// Set the destination
		dtPolyRef dest2, dest3;
		crowd->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getQueryExtents(), crowd->getFilter(), &dest2, 0);
		crowd->getNavMeshQuery()->findNearestPoly(destAgt3, crowd->getQueryExtents(), crowd->getFilter(), &dest3, 0);

		REQUIRE(crowd->requestMoveTarget(indexAgent2, dest2, destAgt2));
		REQUIRE(crowd->requestMoveTarget(indexAgent3, dest3, destAgt3));

		int targets[] = {1, 2};
		dtCrowdAgentParams paramAgt3;
		memcpy(&paramAgt3, &param1, sizeof(dtCrowdAgentParams));
		paramAgt3.steeringBehavior = new dtAlignmentBehavior;
		paramAgt3.alignmentAgents = crowd->getAgents();
		paramAgt3.alignmentTargets = targets;
		paramAgt3.alignmentNbTargets = 2;

		crowd->updateAgentParameters(indexAgent1, &paramAgt3);

		for (int i = 0; i < 10; ++i)
			crowd->update(0.1f, 0);

		// The velocity should not be nil
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) > 0);
		// But not as fast as the others agents
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) < dtVlen(crowd->getAgent(indexAgent3)->vel));


		for (int i = 0; i < 50; ++i)
			crowd->update(0.1f, 0);

		// After a while, the agent should almost be at max speed
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) >= 1.8f);


		for (int i = 0; i < 100; ++i)
			crowd->update(0.1f, 0);

		// After even longer, the agent should have stopped
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) <= 0.2f);

		delete paramAgt3.steeringBehavior;
		paramAgt3.steeringBehavior = 0;
	}

	SECTION("Cohesion Behavior", "With the cohesion behavior, an agent will be attracted to its targets")
	{
		dtCrowdAgentParams param3;
		memcpy(&param3, &param2, sizeof(dtCrowdAgentParams));
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {-5, 0, 1};
		float posAgt3[] = {-5, 0, -1};
		float destAgt2[] = {-5, 0, 1};
		float destAgt3[] = {-5, 0, -1};

		int targets[] = {1, 2};
		param1.steeringBehavior = new dtCohesionBehavior;
		param1.cohesionAgents = crowd->getAgents();
		param1.cohesionTargets = targets;
		param1.cohesionNbTargets = 2;

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1, &param1);
		int indexAgent2 = crowd->addAgent(posAgt2, &param2);
		int indexAgent3 = crowd->addAgent(posAgt3, &param3);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);
		REQUIRE_FALSE(indexAgent3 == -1);

		// Set the destination
		dtPolyRef dest2, dest3;
		crowd->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getQueryExtents(), crowd->getFilter(), &dest2, 0);
		crowd->getNavMeshQuery()->findNearestPoly(destAgt3, crowd->getQueryExtents(), crowd->getFilter(), &dest3, 0);

		REQUIRE(crowd->requestMoveTarget(indexAgent2, dest2, destAgt2));
		REQUIRE(crowd->requestMoveTarget(indexAgent3, dest3, destAgt3));

		for (int i = 0; i < 10; ++i)
			crowd->update(0.1f, 0);

		// The agent should be moving towards the targets
		CHECK(crowd->getAgent(indexAgent1)->npos[0] < 0);

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.1f, 0);

		// After a while, the agent should have moved at the middle of the other 2 agents
		CHECK(crowd->getAgent(indexAgent1)->npos[0] >= -5.1f);
		CHECK(crowd->getAgent(indexAgent1)->npos[0] <= -4.9f);

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.1f, 0);

		// After even longer, the agent should have stopped
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) == 0.f);

		delete param1.steeringBehavior;
		param1.steeringBehavior = 0;
	}

	SECTION("GoTo Behavior", "With the goto behavior, an agent must move towards the given position")
	{
		param1.steeringBehavior = new dtGoToBehavior;

		float posAgt1[] = {0, 0, 0};
		float destAgt1[] = {15, 0, 0};

		param1.gotoTarget = destAgt1;

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1, &param1);

		REQUIRE_FALSE(indexAgent1 == -1);

		crowd->update(2.0, 0);

		// The agent should move to the right
		CHECK(posAgt1[0] < crowd->getAgent(indexAgent1)->npos[0]);

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.01f, 0);

		// The agent should have arrived to its target
		CHECK(crowd->getAgent(indexAgent1)->npos[0] < 15.1f);
		CHECK(crowd->getAgent(indexAgent1)->npos[0] > 14.9f);

		param1.gotoTarget[0] = 25.f;

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.1f, 0);

		// The agent should have reached the boundaries of the navigation mesh
		CHECK(crowd->getAgent(indexAgent1)->npos[0] < 20.1f);
		CHECK(crowd->getAgent(indexAgent1)->npos[0] > 19.6f);

		delete param1.steeringBehavior;
		param1.steeringBehavior = 0;
	}
}

#endif