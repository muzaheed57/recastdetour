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
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene();

	REQUIRE(crowd != 0);

	float posAgt1[] = {0, 0, 0};
	float destAgt1[] = {15, 0, 0};

	// Adding the agent to the crowd
	int indexAgent1 = crowd->addAgent(posAgt1);
	ts.defaultInitializeAgent(*crowd, indexAgent1);

	float correctPosition[] = {19, 0, 0};
	float wrongPosition[] = {100, 0, 10};

	CHECK(crowd->updateAgentPosition(indexAgent1, correctPosition));
	CHECK_FALSE(crowd->updateAgentPosition(indexAgent1, wrongPosition));
}

TEST_CASE("DetourCrowdTest/HashTable", "Testing the hash table containing the pairs <agentID, AgentBehaviorData>")
{
	// Creation of the simulation
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene();

	REQUIRE(crowd != 0);

	// Using a behavior with an empty HT
	dtAlignmentBehavior* align = dtAlignmentBehavior::allocate(0);

	// Adding an element to the HT
	dtAlignmentBehaviorParams* param = align->addBehaviorParams(*crowd->getAgent(0));
	CHECK(param == 0);
	
	// Adding another element
	param = align->addBehaviorParams(*crowd->getAgent(1));
	CHECK(param == 0);
	
	// Adding another element (id is greater or equal to the size of the HT)
	param = align->addBehaviorParams(*crowd->getAgent(2));
	CHECK(param == 0);

	// Trying to access elements from an empty HT
	param = align->getBehaviorParams(*crowd->getAgent(0));
	CHECK_FALSE(param != 0);

	// Getting a none existing parameter
	param = align->getBehaviorParams(*crowd->getAgent(1));
	CHECK_FALSE(param != 0);

	// This the HT is not empty
	dtSeekBehavior* seek = dtSeekBehavior::allocate(2);

	// Adding an element to the HT
	dtSeekBehaviorParams* params = seek->addBehaviorParams(*crowd->getAgent(0));
	CHECK(params != 0);

	// Adding an already existing elements
	params = seek->addBehaviorParams(*crowd->getAgent(0));
	CHECK(params == 0);

	// Adding another element
	params = seek->addBehaviorParams(*crowd->getAgent(1));
	CHECK(params != 0);

	// Adding another element (id is greater or equal to the size of the HT)
	params = seek->addBehaviorParams(*crowd->getAgent(2));
	CHECK(params != 0);

	// Adding new parameters to an already existing element
	dtSeekBehaviorParams* params2 = seek->addBehaviorParams(*crowd->getAgent(0));
	CHECK(params2 == 0);

	// Adding another element (id greater than HT size)
	params = seek->addBehaviorParams(*crowd->getAgent(9));
	CHECK(params != 0);

	dtSeekBehaviorParams* p;
	// Getting an existing parameter
	p = seek->getBehaviorParams(*crowd->getAgent(0));
	CHECK(p != 0);
	// Getting a none existing parameter
	p = seek->getBehaviorParams(*crowd->getAgent(5));
	CHECK_FALSE(p != 0);

	dtSeekBehavior::free(seek);
	dtAlignmentBehavior::free(align);
}

TEST_CASE("DetourCrowdTest/UpdateCrowd", "Test the different ways to update the agents inside a crowd")
{
	// Creation of the simulation
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene();

	REQUIRE(crowd != 0);

	float a1ReferencePosition[3] = {-18, 0, -18};
	float a2ReferencePosition[3] = {18, 0, 18};
	float a3ReferencePosition[3] = {19, 0, -18};
	float a4ReferencePosition[3] = {18, 0, -18};
	float a1Destination[3] = {18, 0, 18};
	float a2Destination[3] = {-18, 0, -18};
	float a3Destination[3] = {1, 0, 0};
	float a4Destination[3] = {0, 0, 0};

	// Adding the agents to the crowd
	int indexAgent1 = crowd->addAgent(a1ReferencePosition);
	int indexAgent2 = crowd->addAgent(a2ReferencePosition);

	REQUIRE_FALSE(indexAgent1 == -1);
	REQUIRE_FALSE(indexAgent2 == -1);

	ts.defaultInitializeAgent(*crowd, indexAgent1);
	ts.defaultInitializeAgent(*crowd, indexAgent2);

	dtPathFollowing* pf1 = dtPathFollowing::allocate(5);
	dtPathFollowingParams* 	params = pf1->addBehaviorParams(*crowd->getAgent(0));
	dtPathFollowingParams* params2 = pf1->addBehaviorParams(*crowd->getAgent(1));

	dtPolyRef poly1, poly2;
	float pos1NavMesh[3], pos2NavMesh[3];
	crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a1ReferencePosition, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly1, pos1NavMesh);
	crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a2ReferencePosition, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly2, pos2NavMesh);
	params->corridor.reset(poly1, pos1NavMesh);
	params2->corridor.reset(poly2, pos2NavMesh);

	pf1->init(*crowd->getCrowdQuery(), ts.getNavMesh(), crowd->getAgents(), crowd->getNbMaxAgents());

	crowd->getAgent(indexAgent1)->behavior = pf1;
	crowd->getAgent(indexAgent2)->behavior = pf1;
		
	// Set the destination
	dtPolyRef dest1, dest2;
	crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a1Destination, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &dest1, 0);
	crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a2Destination, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &dest2, 0);

	REQUIRE(pf1->requestMoveTarget(indexAgent1, dest1, a1Destination));
	REQUIRE(pf1->requestMoveTarget(indexAgent2, dest2, a2Destination));

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

		crowd->update(1.0, &indexAgent1, 1);

        // a1Idx has moved.
        CHECK(!dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));

        // a2Idx hasn't
		CHECK(dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));

		// We just update the second agent
		dtVcopy(a1ReferencePosition, crowd->getAgent(indexAgent1)->npos);

		crowd->update(1.0, &indexAgent2, 1);

        // a1Idx hasn't moved.
        CHECK(dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));

        // a2Idx has
		CHECK(!dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));
		
		// We update every agents using the index indices
		dtVcopy(a2ReferencePosition, crowd->getAgent(indexAgent2)->npos);

		crowd->update(1.0, list, 2);
		
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

			crowd->update(1.0, errorList, 3);

			// a1Idx has moved.
			CHECK(!dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));

			// a2Idx too
			CHECK(!dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));

			// Error Case: the user gives invalid values in the list
			errorList[0] = 999; errorList[1] = -1;
			dtVcopy(a1ReferencePosition, crowd->getAgent(indexAgent1)->npos);
			dtVcopy(a2ReferencePosition, crowd->getAgent(indexAgent2)->npos);

			crowd->update(0.5, errorList, 2);

			// a1Idx hasn't moved.
			CHECK(dtVequal(crowd->getAgent(indexAgent1)->npos, a1ReferencePosition));

			// a2Idx neither
			CHECK(dtVequal(crowd->getAgent(indexAgent2)->npos, a2ReferencePosition));
		}

		// Adding the agents to the crowd
		int indexAgent3 = crowd->addAgent(a3ReferencePosition);
		int indexAgent4 = crowd->addAgent(a4ReferencePosition);

		REQUIRE(indexAgent4 != -1);
		REQUIRE(indexAgent3 != -1);

		ts.defaultInitializeAgent(*crowd, indexAgent3);
		ts.defaultInitializeAgent(*crowd, indexAgent4);

		// Set the destinations
		dtPolyRef a3TargetRef;
		dtPolyRef a4TargetRef;
		dtCrowdQuery* query = crowd->getCrowdQuery();
		query->getNavMeshQuery()->findNearestPoly(a3Destination, query->getQueryExtents(), query->getQueryFilter(), &a3TargetRef, a3Destination);
		query->getNavMeshQuery()->findNearestPoly(a4Destination, query->getQueryExtents(), query->getQueryFilter(), &a4TargetRef, a4Destination);

		dtVcopy(a1ReferencePosition, crowd->getAgent(indexAgent1)->npos);
		dtVcopy(a2ReferencePosition, crowd->getAgent(indexAgent2)->npos);
		dtVcopy(a3ReferencePosition, crowd->getAgent(indexAgent3)->npos);
		dtVcopy(a4ReferencePosition, crowd->getAgent(indexAgent4)->npos);

		pf1->init(*crowd->getCrowdQuery(), ts.getNavMesh(), crowd->getAgents(), crowd->getNbMaxAgents());
		dtPathFollowingParams* params3 = pf1->addBehaviorParams(*crowd->getAgent(indexAgent3));
		dtPathFollowingParams* params4 = pf1->addBehaviorParams(*crowd->getAgent(indexAgent4));

		crowd->getAgent(indexAgent3)->behavior = pf1;
		crowd->getAgent(indexAgent4)->behavior = pf1;
		
		// Set the corridor
		dtPolyRef poly1, poly2, poly3, poly4;
		float pos1NavMesh[3], pos2NavMesh[3], pos3NavMesh[3], pos4NavMesh[3];
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a1ReferencePosition, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly1, pos1NavMesh);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a2ReferencePosition, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly2, pos2NavMesh);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a3ReferencePosition, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly3, pos3NavMesh);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a4ReferencePosition, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly4, pos4NavMesh);
		params->corridor.reset(poly1, pos1NavMesh);
		params2->corridor.reset(poly2, pos2NavMesh);
		params3->corridor.reset(poly3, pos3NavMesh);
		params4->corridor.reset(poly4, pos4NavMesh);

		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a1Destination, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&params->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a2Destination, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&params2->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a3Destination, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&params3->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(a4Destination, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&params4->targetRef, 0);

		REQUIRE(pf1->requestMoveTarget(indexAgent1, params->targetRef, a1Destination));
		REQUIRE(pf1->requestMoveTarget(indexAgent2, params2->targetRef, a2Destination));
		REQUIRE(pf1->requestMoveTarget(indexAgent3, params3->targetRef, a3Destination));
		REQUIRE(pf1->requestMoveTarget(indexAgent4, params4->targetRef, a4Destination));
		
		crowd->removeAgent(indexAgent1);
		crowd->update(0.5);

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
		float posAgt3[3] = {crowd->getAgent(indexAgent1)->npos[0] + 1, 
							0, 
							crowd->getAgent(indexAgent1)->npos[2] + 1};
		float destAgt3[3] = {posAgt3[0], posAgt3[1], posAgt3[2]};

		int indexAgent3 = crowd->addAgent(posAgt3);

		REQUIRE_FALSE(indexAgent3 == -1);

		ts.defaultInitializeAgent(*crowd, indexAgent3);
		crowd->getAgent(indexAgent3)->behavior = pf1;

		// We update the environment only for the third agent
		int index = 2;
		crowd->updateEnvironment(&index, 1);
		const dtCrowdAgentEnvironment* env = crowd->getAgentsEnvironment();

		// The third agent has been placed next to another agent, and therefore it should have one neighbor
		CHECK(env[indexAgent3].nbNeighbors == 1);
		// The other agents on the other hand haven't updated their environment, and therefore should have no neighbors
		CHECK(env[indexAgent1].nbNeighbors == 0);

		// Now we update everyone's environment, thus every agent has one neighbor
		crowd->updateEnvironment();
		CHECK(env[indexAgent3].nbNeighbors == 1);
		CHECK(env[indexAgent1].nbNeighbors == 1);

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

	dtPathFollowing::free(pf1);
}

TEST_CASE("DetourCrowdTest/InitCrowd", "Test whether the initialization of a crowd is successful")
{
	dtCrowd crowd;

	REQUIRE(crowd.getAgentCount() == 0);
}
