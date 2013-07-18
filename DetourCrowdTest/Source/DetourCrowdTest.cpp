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

#include "DetourCrowdTestUtils.h"

#include "DetourAlignmentBehavior.h"
#include "DetourPathFollowing.h"
#include "DetourSeekBehavior.h"

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

TEST_CASE("DetourCrowdTest/UpdateAgentPosition", "We want to dynamically update the position of an agent with error checking")
{
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene(20, 0.5f);

	REQUIRE(crowd != 0);

	float posAgt1[] = {0, 0, 0};
	dtCrowdAgent ag;

	// Adding the agent to the crowd
	REQUIRE(crowd->addAgent(ag, posAgt1));
	ts.defaultInitializeAgent(*crowd, ag.id);

	float correctPosition[] = {19, 0, 0};
	float wrongPosition[] = {100, 0, 10};

	CHECK(crowd->updateAgentPosition(ag.id, correctPosition));
	CHECK_FALSE(crowd->updateAgentPosition(ag.id, wrongPosition));
}

TEST_CASE("DetourCrowdTest/HashTable", "Testing the hash table containing the pairs <agentID, AgentBehaviorData>")
{
	// Creation of the simulation
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene(10, 0.5f);

	REQUIRE(crowd != 0);

	// Using a behavior with an empty HT
	dtAlignmentBehavior* align = dtAlignmentBehavior::allocate(0);

	// Adding an element to the HT
	dtAlignmentBehaviorParams* param = align->getBehaviorParams(crowd->getAgent(0)->id);
	CHECK(param == 0);
	
	// Adding another element
	param = align->getBehaviorParams(crowd->getAgent(1)->id);
	CHECK(param == 0);
	
	// Adding another element (id is greater or equal to the size of the HT)
	param = align->getBehaviorParams(crowd->getAgent(2)->id);
	CHECK(param == 0);

	// This time the HT is not empty
	dtSeekBehavior* seek = dtSeekBehavior::allocate(2);

	// Adding an element to the HT
	dtSeekBehaviorParams* params = seek->getBehaviorParams(crowd->getAgent(0)->id);
	CHECK(params != 0);

	// Getting an already existing elements
	params = seek->getBehaviorParams(crowd->getAgent(0)->id);
	CHECK(params != 0);

	// Adding another element
	params = seek->getBehaviorParams(crowd->getAgent(1)->id);
	CHECK(params != 0);

	// Adding another element (id is greater or equal to the size of the HT)
	params = seek->getBehaviorParams(crowd->getAgent(2)->id);
	CHECK(params != 0);

	// Adding another element (id greater than HT size)
	params = seek->getBehaviorParams(crowd->getAgent(9)->id);
	CHECK(params != 0);

	dtSeekBehavior::free(seek);
	dtAlignmentBehavior::free(align);
}

TEST_CASE("DetourCrowdTest/UpdateCrowd", "Test the different ways to update the agents inside a crowd")
{
	// Creation of the simulation
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene(4, 0.5f);

	REQUIRE(crowd != 0);

	float posAgt1[] = {-18, 0, -18};
	float posAgt2[] = {18, 0, 18};
	float posAgt3[] = {-18, 0, 18};
	float posAgt4[] = {18, 0, -18};
	float destAgt1[] = {18, 0, 18};
	float destAgt2[] = {-18, 0, -18};
	float destAgt3[] = {18, 0, -18};
	float destAgt4[] = {-18, 0, 18};
	dtCrowdAgent ag1, ag2;

	// Adding the agents to the crowd
	REQUIRE(crowd->addAgent(ag1, posAgt1));
	REQUIRE(crowd->addAgent(ag2, posAgt2));

	ts.defaultInitializeAgent(*crowd, ag1.id);
	ts.defaultInitializeAgent(*crowd, ag2.id);

	dtPathFollowing* pf1 = dtPathFollowing::allocate(5);
	dtPathFollowingParams* 	params = pf1->getBehaviorParams(crowd->getAgent(0)->id);
	dtPathFollowingParams* params2 = pf1->getBehaviorParams(crowd->getAgent(1)->id);
	params->init(256);
	params2->init(256);
	params->preparePath(crowd->getAgent(0)->position, *crowd->getCrowdQuery());
	params2->preparePath(crowd->getAgent(1)->position, *crowd->getCrowdQuery());

	pf1->init(*crowd->getCrowdQuery());

	crowd->setAgentBehavior(ag1.id, pf1);
	crowd->setAgentBehavior(ag2.id, pf1);
		
	// Set the destination
	dtPolyRef dest1, dest2;
	crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &dest1, 0);
	crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &dest2, 0);

	pf1->requestMoveTarget(ag1.id, dest1, destAgt1);
	pf1->requestMoveTarget(ag2.id, dest2, destAgt2);

	SECTION("Edit Agents", "Acessing and modifying agents through the dtCrowd interface")
	{
		dtCrowdAgent ag;
		ag.maxAcceleration = 999;

		// Index is invalid, the agent should not have been updated
		crowd->fetchAgent(ag, -1);
		CHECK(ag.maxAcceleration == 999);

		// Index is valid, the agent should have been updated
		crowd->fetchAgent(ag, ag1.id);
		CHECK(ag.maxAcceleration == 10.f);

		// Modifying the agent's radius
		ag.radius = 9.f;
		ag.id = -1;

		// Applying modifications should not work since the id is invalid
		CHECK_FALSE(crowd->applyAgent(ag));

		ag.id = ag1.id;

		// Applying modifications should work since the id is valid
		CHECK(crowd->applyAgent(ag));

		// Now the property of the modified agent should have changed
		CHECK(crowd->getAgent(ag1.id)->radius == 9.f);
	}

	SECTION("Update all agents", "We update every agent at the time")
	{
		crowd->update(1.0, 0);

		float agt1NewPos[3], agt2NewPos[3];

		dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);
		dtVcopy(agt2NewPos, crowd->getAgent(ag2.id)->position);		

		CHECK((!dtVequal(posAgt1, agt1NewPos) && !dtVequal(posAgt2, agt2NewPos)));
	}

	SECTION("Update specific agents", "We just want to update some specific agents")
	{
		// We just update the first agent
		unsigned list[] = {ag1.id, ag2.id};
		unsigned listSize = 1;
		float agt1NewPos[3], agt2NewPos[3];

		dtVcopy(posAgt1, crowd->getAgent(ag1.id)->position);
		dtVcopy(posAgt2, crowd->getAgent(ag2.id)->position);

		crowd->update(1.0, list, 1);

		dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);
		dtVcopy(agt2NewPos, crowd->getAgent(ag2.id)->position);

		CHECK((!dtVequal(posAgt1, agt1NewPos) && dtVequal(posAgt2, agt2NewPos)));

		// We just update the second agent
		dtVcopy(posAgt1, agt1NewPos);

		list[0] = ag2.id;

        crowd->update(1.0, list, listSize);

		dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);
		dtVcopy(agt2NewPos, crowd->getAgent(ag2.id)->position);

		CHECK((dtVequal(posAgt1, agt1NewPos) && !dtVequal(posAgt2, agt2NewPos)));
		
		// We update every agent using the index list
		dtVcopy(posAgt2, agt2NewPos);
		
		listSize = 2;
		list[0] = ag1.id;

		crowd->update(1.0, list, listSize);

		dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);
		dtVcopy(agt2NewPos, crowd->getAgent(ag2.id)->position);
		
		CHECK((!dtVequal(posAgt1, agt1NewPos) && !dtVequal(posAgt2, agt2NewPos)));

		SECTION("Error cases", "Error cases when updating specific agents")
		{
			// Error case: the user gives a list of indexes that is too big
			// Size is 3, but there are only 2 agents
			unsigned errorList[3] = {0, 1, 2};

			crowd->update(1.0, errorList, 3);

			// a1Idx has moved.
			CHECK(!dtVequal(crowd->getAgent(ag1.id)->position, agt1NewPos));

			// a2Idx too
			CHECK(!dtVequal(crowd->getAgent(ag2.id)->position, agt2NewPos));

			// Error Case: the user gives invalid values in the list
			errorList[0] = 999; errorList[1] = -1;
			dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);
			dtVcopy(agt2NewPos, crowd->getAgent(ag2.id)->position);

			crowd->update(0.5, errorList, 2);

			// a1Idx hasn't moved.
			CHECK(dtVequal(crowd->getAgent(ag1.id)->position, agt1NewPos));

			// a2Idx neither
			CHECK(dtVequal(crowd->getAgent(ag2.id)->position, agt2NewPos));
		}

		// Adding the agents to the crowd
		dtCrowdAgent ag3, ag4;

		REQUIRE(crowd->addAgent(ag3, posAgt3));
		REQUIRE(crowd->addAgent(ag4, posAgt4));

		ts.defaultInitializeAgent(*crowd, ag3.id);
		ts.defaultInitializeAgent(*crowd, ag4.id);

		// Set the destinations
		dtPolyRef a3TargetRef;
		dtPolyRef a4TargetRef;
		dtCrowdQuery* query = crowd->getCrowdQuery();
		query->getNavMeshQuery()->findNearestPoly(destAgt3, query->getQueryExtents(), query->getQueryFilter(), &a3TargetRef, destAgt3);
		query->getNavMeshQuery()->findNearestPoly(destAgt4, query->getQueryExtents(), query->getQueryFilter(), &a4TargetRef, destAgt4);

		dtVcopy(posAgt1, crowd->getAgent(ag1.id)->position);
		dtVcopy(posAgt2, crowd->getAgent(ag2.id)->position);
		dtVcopy(posAgt3, crowd->getAgent(ag3.id)->position);
		dtVcopy(posAgt4, crowd->getAgent(ag4.id)->position);

		pf1->init(*crowd->getCrowdQuery());
		dtPathFollowingParams* params3 = pf1->getBehaviorParams(ag3.id);
		dtPathFollowingParams* params4 = pf1->getBehaviorParams(ag4.id);
		params3->init(256);
		params4->init(256);
		params3->preparePath(crowd->getAgent(ag3.id)->position, *crowd->getCrowdQuery());
		params4->preparePath(crowd->getAgent(ag4.id)->position, *crowd->getCrowdQuery());

		crowd->setAgentBehavior(ag3.id, pf1);
		crowd->setAgentBehavior(ag4.id, pf1);

		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&params->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&params2->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt3, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&params3->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt4, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&params4->targetRef, 0);

		REQUIRE(pf1->requestMoveTarget(ag1.id, params->targetRef, destAgt1));
		REQUIRE(pf1->requestMoveTarget(ag2.id, params2->targetRef, destAgt2));
		REQUIRE(pf1->requestMoveTarget(ag3.id, params3->targetRef, destAgt3));
		REQUIRE(pf1->requestMoveTarget(ag4.id, params4->targetRef, destAgt4));
		
		crowd->removeAgent(ag1.id);
		crowd->update(0.5);

		// Every agent has moved but the removed one
		CHECK(dtVequal(crowd->getAgent(ag1.id)->position, posAgt1));
		CHECK(!dtVequal(crowd->getAgent(ag2.id)->position, posAgt2));
		CHECK(!dtVequal(crowd->getAgent(ag3.id)->position, posAgt3));
		CHECK(!dtVequal(crowd->getAgent(ag4.id)->position, posAgt4));		
	}

	SECTION("Separate velocity and position", "We want to be able to update the velocity and the position of the agents separatly")
	{
		// Update only the environment
		// The velocity and the position must not be changed.
		float agt1NewPos[3], agt2NewPos[3], agt1NewVel[3], agt2NewVel[3];
		float velAgt1[3], velAgt2[3];
		
		dtVcopy(posAgt1, crowd->getAgent(ag1.id)->position);
		dtVcopy(posAgt2, crowd->getAgent(ag2.id)->position);
		dtVcopy(velAgt1, crowd->getAgent(ag1.id)->velocity);
		dtVcopy(velAgt2, crowd->getAgent(ag2.id)->velocity);

		crowd->updateEnvironment();

		dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);
		dtVcopy(agt2NewPos, crowd->getAgent(ag2.id)->position);
		dtVcopy(agt1NewVel, crowd->getAgent(ag1.id)->velocity);
		dtVcopy(agt2NewVel, crowd->getAgent(ag2.id)->velocity);

		REQUIRE((dtVequal(posAgt1, agt1NewPos) && dtVequal(posAgt2, agt2NewPos) && 
			     dtVequal(velAgt1, agt1NewVel) && dtVequal(velAgt2, agt2NewVel)));

		// Checking if the neighbourhood of an agent is correctly updated
		// For that we create another agent next to an already existing agent.
		float posAgt3[3] = {crowd->getAgent(ag1.id)->position[0] + 1, 
							0, 
							crowd->getAgent(ag1.id)->position[2] + 1};
		dtCrowdAgent ag3;

		REQUIRE(crowd->addAgent(ag3, posAgt3));

		ts.defaultInitializeAgent(*crowd, ag3.id);

		crowd->setAgentBehavior(ag3.id, pf1);

		// We update the environment only for the third agent
		unsigned index = 2;
		crowd->updateEnvironment(&index, 1);

		// The third agent has been placed next to another agent, and therefore it should have one neighbor
		CHECK(crowd->getAgentEnvironment(ag3.id)->nbNeighbors == 1);
		// The other agents on the other hand haven't updated their environment, and therefore should have no neighbors
		CHECK(crowd->getAgentEnvironment(ag1.id)->nbNeighbors == 0);

		// Now we update everyone's environment, thus every agent has one neighbor
		crowd->updateEnvironment();
		CHECK(crowd->getAgentEnvironment(ag3.id)->nbNeighbors == 1);
		CHECK(crowd->getAgentEnvironment(ag1.id)->nbNeighbors == 1);

		crowd->removeAgent(ag3.id);

		// Update velocity only
		crowd->updateVelocity(0.2f, 0);

		dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);
		dtVcopy(agt2NewPos, crowd->getAgent(ag2.id)->position);
		dtVcopy(agt1NewVel, crowd->getAgent(ag1.id)->velocity);
		dtVcopy(agt2NewVel, crowd->getAgent(ag2.id)->velocity);

        // velocities should have changed
        CHECK(!dtVequal(crowd->getAgent(ag1.id)->velocity, velAgt1));
        CHECK(!dtVequal(crowd->getAgent(ag2.id)->velocity, velAgt2));

		// Update position only
		dtVcopy(velAgt1, crowd->getAgent(ag1.id)->velocity);
		dtVcopy(velAgt2, crowd->getAgent(ag2.id)->velocity);

		crowd->updatePosition(0.2f);

		dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);
		dtVcopy(agt2NewPos, crowd->getAgent(ag2.id)->position);
		dtVcopy(agt1NewVel, crowd->getAgent(ag1.id)->velocity);
		dtVcopy(agt2NewVel, crowd->getAgent(ag2.id)->velocity);

        // velocities should be the same changed
        CHECK(dtVequal(crowd->getAgent(ag1.id)->velocity, velAgt1));
        CHECK(dtVequal(crowd->getAgent(ag2.id)->velocity, velAgt2));	
	}

	dtPathFollowing::free(pf1);
}

TEST_CASE("DetourCrowdTest/InitCrowd", "Test whether the initialization of a crowd is successful")
{
	dtCrowd crowd;

	REQUIRE(crowd.getAgentCount() == 0);
}
