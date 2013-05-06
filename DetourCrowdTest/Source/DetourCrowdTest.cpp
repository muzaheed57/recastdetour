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

#include "DetourBehaviorsTests.h"

TEST_CASE("DetourCrowdTest/UpdateAgentPosition", "We want to dynamically update the position of an agent with error checking")
{
	// Creation of a square
	float* vert = new float[12];
	int* tri = new int[6];

	vert[0] = 20.0; vert[1] = 0.0; vert[2] = 20.0;
	vert[3] = 20.0; vert[4] = 0.0; vert[5] = -20.0;
	vert[6] = -20.0; vert[7] = 0.0; vert[8] = -20.0;
	vert[9] = -20.0; vert[10] = 0.0; vert[11] = 20.0;

	tri[0] = 0; tri[1] = 1; tri[2] = 2;
	tri[3] = 2; tri[4] = 3; tri[5] = 0;

	dtCrowdAgentParams param1;
	TestScene ts;
	dtCrowd* crowd = ts.createScene(param1, vert, tri);

	REQUIRE(crowd != 0);

	float posAgt1[] = {0, 0, 0};
	float destAgt1[] = {15, 0, 0};

	// Adding the agent to the crowd
	int indexAgent1 = crowd->addAgent(posAgt1, &param1);

	float correctPosition[] = {19, 0, 0};
	float wrongPosition[] = {100, 0, 10};

	CHECK(crowd->updateAgentPosition(indexAgent1, correctPosition));
	CHECK_FALSE(crowd->updateAgentPosition(indexAgent1, wrongPosition));
}

TEST_CASE("DetourCrowdTest/UpdateCrowd", "Test the different ways to update the agents inside a crowd")
{
	// Creation of a square
	float* vert = new float[12];
	int* tri = new int[6];

	vert[0] = 20.0; vert[1] = 0.0; vert[2] = 20.0;
	vert[3] = 20.0; vert[4] = 0.0; vert[5] = -20.0;
	vert[6] = -20.0; vert[7] = 0.0; vert[8] = -20.0;
	vert[9] = -20.0; vert[10] = 0.0; vert[11] = 20.0;

	tri[0] = 0; tri[1] = 1; tri[2] = 2;
	tri[3] = 2; tri[4] = 3; tri[5] = 0;

	// Creation of the simulation
	dtCrowdAgentParams param1, param2;
	TestScene ts;
	dtCrowd* crowd = ts.createScene(param1, vert, tri);

	REQUIRE(crowd != 0);

	memcpy(&param2, &param1, sizeof(dtCrowdAgentParams));

	float a1ReferencePosition[3] = {-18, 0, -18};
	float a2ReferencePosition[3] = {18, 0, 18};
	float a3ReferencePosition[3] = {19, 0, -18};
	float a4ReferencePosition[3] = {18, 0, -18};
	float a1Destination[3] = {18, 0, 18};
	float a2Destination[3] = {-18, 0, -18};
	float a3Destination[3] = {1, 0, 0};
	float a4Destination[3] = {0, 0, 0};

	dtPathFollowing* pf1 = dtAllocBehavior<dtPathFollowing>();
	dtPathFollowing* pf2 = dtAllocBehavior<dtPathFollowing>();
	pf1->init(crowd->getPathQueue(), crowd->getNavMeshQuery(), crowd->getQueryExtents(), crowd->getEditableFilter(), crowd->getMaxPathResult(), 
		crowd->getAgents(), crowd->getNbMaxAgents(), crowd->getAnims());
	pf2->init(crowd->getPathQueue(), crowd->getNavMeshQuery(), crowd->getQueryExtents(), crowd->getEditableFilter(), crowd->getMaxPathResult(), 
		crowd->getAgents(), crowd->getNbMaxAgents(), crowd->getAnims());
	param1.steeringBehavior = pf1;
	param2.steeringBehavior = pf2;

	// Adding the agents to the crowd
	int indexAgent1 = crowd->addAgent(a1ReferencePosition, &param1);
	int indexAgent2 = crowd->addAgent(a2ReferencePosition, &param2);

	REQUIRE_FALSE(indexAgent1 == -1);
	REQUIRE_FALSE(indexAgent2 == -1);

	// Set the destination
	crowd->getNavMeshQuery()->findNearestPoly(a1Destination, crowd->getQueryExtents(), crowd->getFilter(), &crowd->getAgent(indexAgent1)->targetRef, 0);
	crowd->getNavMeshQuery()->findNearestPoly(a2Destination, crowd->getQueryExtents(), crowd->getFilter(), &crowd->getAgent(indexAgent2)->targetRef, 0);

	REQUIRE(crowd->requestMoveTarget(indexAgent1, crowd->getAgent(indexAgent1)->targetRef, a1Destination));
	REQUIRE(crowd->requestMoveTarget(indexAgent2, crowd->getAgent(indexAgent2)->targetRef, a2Destination));

	SECTION("Update all agents", "We update every agent at the time")
	{
		crowd->update(1.0, 0);

        CHECK(!dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));
		CHECK(!dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));
	}

	SECTION("Update specific agents", "We just want to update some specific agents")
	{
		// We just update the first agent
        dtVcopy(a1ReferencePosition, crowd->getAgent(indexAgent1)->npos);
        dtVcopy(a2ReferencePosition, crowd->getAgent(indexAgent2)->npos);
		int list[] = {indexAgent1, indexAgent2};
		unsigned listSize = 1;
		float agt1NewPos[3], agt2NewPos[3];

		crowd->update(1.0, 0, &indexAgent1, 1);

        // a1Idx has moved.
        CHECK(!dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));

        // a2Idx hasn't
		CHECK(dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));

		// We just update the second agent
		dtVcopy(a1ReferencePosition, crowd->getAgent(indexAgent1)->npos);

		crowd->update(1.0, 0, &indexAgent2, 1);

        // a1Idx hasn't moved.
        CHECK(dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));

        // a2Idx has
		CHECK(!dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));
		
		// We update every agents using the index indices
		dtVcopy(a2ReferencePosition, crowd->getAgent(indexAgent2)->npos);

		crowd->update(1.0, 0, list, 2);
		
		// a1Idx has moved.
        CHECK(!dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));
		list[0] = indexAgent1;

        // a2Idx too
		CHECK(!dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));

		dtVcopy(a1ReferencePosition, crowd->getAgent(indexAgent1)->npos);
		dtVcopy(a2ReferencePosition, crowd->getAgent(indexAgent2)->npos);

		SECTION("Error cases", "Error cases when updating specific agents")
		{
			// Error case: the user gives a list of indexes that is too big
			// Size is 3, but there are only 2 agents
			int errorList[3] = {0, 1, 2};

			crowd->update(1.0, 0, errorList, 3);

			// a1Idx has moved.
			CHECK(!dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));

			// a2Idx too
			CHECK(!dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));

			// Error Case: the user gives invalid values in the list
			errorList[0] = 999; errorList[1] = -1;
			dtVcopy(a1ReferencePosition, crowd->getAgent(indexAgent1)->npos);
			dtVcopy(a2ReferencePosition, crowd->getAgent(indexAgent2)->npos);

			crowd->update(0.5, 0, errorList, 2);

			// a1Idx hasn't moved.
			CHECK(dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));

			// a2Idx neither
			CHECK(dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));
		}

		// Adding the agents to the crowd
		int indexAgent3 = crowd->addAgent(a3ReferencePosition, &param1);
		int indexAgent4 = crowd->addAgent(a4ReferencePosition, &param2);

		// Set the destinations
		dtPolyRef a3TargetRef;
		dtPolyRef a4TargetRef;
		crowd->getNavMeshQuery()->findNearestPoly(a3Destination, crowd->getQueryExtents(), crowd->getFilter(), &a3TargetRef, a3Destination);
		crowd->getNavMeshQuery()->findNearestPoly(a4Destination, crowd->getQueryExtents(), crowd->getFilter(), &a4TargetRef, a4Destination);

		REQUIRE(crowd->requestMoveTarget(indexAgent3, a3TargetRef, a3Destination));
		REQUIRE(crowd->requestMoveTarget(indexAgent4, a4TargetRef, a4Destination));

		REQUIRE(indexAgent4 != -1);
		REQUIRE(indexAgent3 != -1);

		dtVcopy(a1ReferencePosition, crowd->getAgent(indexAgent1)->npos);
		dtVcopy(a2ReferencePosition, crowd->getAgent(indexAgent2)->npos);
		dtVcopy(a3ReferencePosition, crowd->getAgent(indexAgent3)->npos);
		dtVcopy(a4ReferencePosition, crowd->getAgent(indexAgent4)->npos);

		crowd->removeAgent(indexAgent1);
		crowd->update(0.5, 0);

		// Every agent has moved but the removed one
		CHECK(dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));
		CHECK(!dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));
		CHECK(!dtVequal(crowd->getAgent(indexAgent3)->npos, a3ReferencePosition));
		CHECK(!dtVequal(crowd->getAgent(indexAgent4)->npos, a4ReferencePosition));		
	}

	SECTION("Separate velocity and position", "We want to be able to update the velocity and the position of the agents separatly")
	{
		// Update only the environment
		// The velocity and the position must not be changed.

		float agt1NewPos[3], agt2NewPos[3], agt1NewVel[3], agt2NewVel[3];
		float a1ReferenceVelocity[3], a2ReferenceVelocity[3];
		dtVcopy(a1ReferencePosition,crowd->getAgent(indexAgent1)->npos);
		dtVcopy(a2ReferencePosition,crowd->getAgent(indexAgent2)->npos);
		dtVcopy(a1ReferenceVelocity, crowd->getAgent(indexAgent1)->vel);
		dtVcopy(a2ReferenceVelocity, crowd->getAgent(indexAgent2)->vel);

		crowd->updateEnvironment();

		dtVcopy(agt1NewPos, crowd->getAgent(indexAgent1)->npos);
		dtVcopy(agt2NewPos, crowd->getAgent(indexAgent2)->npos);
		dtVcopy(agt1NewVel, crowd->getAgent(indexAgent1)->vel);
		dtVcopy(agt2NewVel, crowd->getAgent(indexAgent2)->vel);

		REQUIRE((dtVequal(a1ReferencePosition, agt1NewPos) && dtVequal(a2ReferencePosition, agt2NewPos) && 
			     dtVequal(a1ReferenceVelocity, agt1NewVel) && dtVequal(a2ReferenceVelocity, agt2NewVel)));

		// Checking if the neighbourhood of an agent is correctly updated
		// For that we create another agent next to an already existing agent.
		dtCrowdAgent ag3;
		dtCrowdAgentParams param3;
		float posAgt3[3] = {crowd->getAgent(indexAgent1)->npos[0] + 1, 
							0, 
							crowd->getAgent(indexAgent1)->npos[2] + 1};
		float destAgt3[3] = {posAgt3[0], posAgt3[1], posAgt3[2]};

		memcpy(&param3, &param1, sizeof(dtCrowdAgentParams));
		int indexAgent3 = crowd->addAgent(posAgt3, &param3);

		// We update the environment only for the third agent
		int index = 2;
		crowd->updateEnvironment(&index, 1);

		// The third agent has been placed next to another agent, and therefore it should have one neighbor
		CHECK(crowd->getAgent(indexAgent3)->nneis == 1);
		// The other agents on the other hand haven't updated their environment, and therefore should have no neighbors
		CHECK(crowd->getAgent(indexAgent1)->nneis == 0);

		// Now we update everyone's environment, thus every agent has one neighbor
		crowd->updateEnvironment();
		CHECK(crowd->getAgent(indexAgent3)->nneis == 1);
		CHECK(crowd->getAgent(indexAgent1)->nneis == 1);

		crowd->removeAgent(indexAgent3);

		// Update velocity only
		crowd->updateVelocity(0.2f, 0);

        // positions should be the same.
        CHECK(dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));
        CHECK(dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));

        // velocities should have changed
        CHECK(!dtVequal(crowd->getAgent(indexAgent1)->vel, a1ReferenceVelocity));
        CHECK(!dtVequal(crowd->getAgent(indexAgent2)->vel, a2ReferenceVelocity));

		// Update position only
		dtVcopy(a1ReferenceVelocity, crowd->getAgent(indexAgent1)->vel);
		dtVcopy(a2ReferenceVelocity, crowd->getAgent(indexAgent2)->vel);

		crowd->updatePosition(0.2f);

		// positions should have changed.
        CHECK(!dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));
        CHECK(!dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));

        // velocities should be the same changed
        CHECK(dtVequal(crowd->getAgent(indexAgent1)->vel, a1ReferenceVelocity));
        CHECK(dtVequal(crowd->getAgent(indexAgent2)->vel, a2ReferenceVelocity));	
	}

	dtFreeBehavior<dtPathFollowing>(pf1);
	dtFreeBehavior<dtPathFollowing>(pf2);
}

TEST_CASE("DetourCrowdTest/InitCrowd", "Test whether the initialization of a crowd is successful")
{
	dtCrowd crowd;

	REQUIRE(crowd.getAgentCount() == 0);
}
