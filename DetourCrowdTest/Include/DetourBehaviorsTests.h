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

#include "DetourPipelineTest.h"

#include <cstring>


TEST_CASE("DetourCrowdTest/CustomBehavior", "Test whether the custom behaviors behave correctly")
{
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene();
	REQUIRE(crowd != 0);
	
	SECTION("Seek Behavior", "With the seeking behavior, an agent must move towards its target")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {15, 0, 0};
		float destAgt2[] = {15, 0, 0};

		dtSeekBehavior* seek = dtSeekBehavior::allocate(5);
		dtSeekBehaviorParams* params = seek->addBehaviorParams(*crowd->getAgent(0));
		params->seekTarget = crowd->getAgent(1);
		params->seekDistance = 0;
		params->seekPredictionFactor = 0;

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1);
		int indexAgent2 = crowd->addAgent(posAgt2);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);

		ts.defaultInitializeAgent(*crowd, indexAgent1);
		ts.defaultInitializeAgent(*crowd, indexAgent2);

		crowd->getAgent(indexAgent1)->behavior = seek;
				
		crowd->update(2.0, 0);

		float agt1NewPos[3];

		dtVcopy(agt1NewPos, crowd->getAgent(indexAgent1)->npos);	

		// The first agent should move to the right (since he is seeking the second agent)
		CHECK(posAgt1[0] < crowd->getAgent(indexAgent2)->npos[0]);

		dtSeekBehavior::free(seek);
		crowd->getAgent(indexAgent1)->behavior = 0;
	}

	SECTION("Flocking Behavior", "Unit tests about the flocking behavior")
	{
		dtPathFollowing* pf = dtPathFollowing::allocate(1);
		dtFlockingBehavior* flocking = dtFlockingBehavior::allocate(5, 2, 1, 1);

		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {1, 0, 1};
		float posAgt3[] = {1.5f, 0, 1};
		float posAgt4[] = {1, 0, 1.5f};
		float posLeader[] = {-0.5f, 0, -0.5f};
		float destLeader[] = {-14, 0, -14};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1);
		int indexAgent2 = crowd->addAgent(posAgt2);
		int indexAgent3 = crowd->addAgent(posAgt3);
		int indexAgent4 = crowd->addAgent(posAgt4);
		int indexLeader = crowd->addAgent(posLeader);

		dtPathFollowingParams* pfParams = pf->addBehaviorParams(*crowd->getAgent(4));

		dtFlockingBehaviorParams* flockParams = flocking->addBehaviorParams(*crowd->getAgent(0));
		dtFlockingBehaviorParams* flockParams2 = flocking->addBehaviorParams(*crowd->getAgent(1));
		dtFlockingBehaviorParams* flockParams3 = flocking->addBehaviorParams(*crowd->getAgent(2));
		dtFlockingBehaviorParams* flockParams4 = flocking->addBehaviorParams(*crowd->getAgent(3));

		dtPolyRef polyLeader;
		float posLeaderNavMesh[3];
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posLeader, crowd->getCrowdQuery()->getQueryExtents(), 
			crowd->getCrowdQuery()->getQueryFilter(), &polyLeader, posLeaderNavMesh);
		pfParams->corridor.reset(polyLeader, posLeaderNavMesh);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);
		REQUIRE_FALSE(indexAgent3 == -1);
		REQUIRE_FALSE(indexAgent4 == -1);
		REQUIRE_FALSE(indexLeader == -1);

		ts.defaultInitializeAgent(*crowd, indexAgent1);
		ts.defaultInitializeAgent(*crowd, indexAgent2);
		ts.defaultInitializeAgent(*crowd, indexAgent3);
		ts.defaultInitializeAgent(*crowd, indexAgent4);
		ts.defaultInitializeAgent(*crowd, indexLeader);

		crowd->getAgent(indexAgent1)->behavior = flocking;
		crowd->getAgent(indexAgent2)->behavior = flocking;
		crowd->getAgent(indexAgent3)->behavior = flocking;
		crowd->getAgent(indexAgent4)->behavior = flocking;
		
		flockParams->agents = crowd->getAgents();
		flockParams->separationDistance = 2;
		flockParams2->agents = crowd->getAgents();
		flockParams2->separationDistance = 2;
		flockParams3->agents = crowd->getAgents();
		flockParams3->separationDistance = 2;
		flockParams4->agents = crowd->getAgents();
		flockParams4->separationDistance = 2;
		flockParams->nbflockingTargets = 4;
		flockParams2->nbflockingTargets = 4;
		flockParams3->nbflockingTargets = 4;
		flockParams4->nbflockingTargets = 4;

		int* toFlockWith1 = (int*) dtAlloc(sizeof(int) * flockParams->nbflockingTargets, DT_ALLOC_TEMP);
		int* toFlockWith2 = (int*) dtAlloc(sizeof(int) * flockParams2->nbflockingTargets, DT_ALLOC_TEMP);
		int* toFlockWith3 = (int*) dtAlloc(sizeof(int) * flockParams3->nbflockingTargets, DT_ALLOC_TEMP);
		int* toFlockWith4 = (int*) dtAlloc(sizeof(int) * flockParams4->nbflockingTargets, DT_ALLOC_TEMP);

		toFlockWith1[0] = 1; toFlockWith1[1] = 2; toFlockWith1[2] = 3; toFlockWith1[3] = 4;
		toFlockWith2[0] = 0; toFlockWith2[1] = 2; toFlockWith2[2] = 3; toFlockWith2[3] = 4;
		toFlockWith3[0] = 0; toFlockWith3[1] = 1; toFlockWith3[2] = 3; toFlockWith3[3] = 4;
		toFlockWith4[0] = 0; toFlockWith4[1] = 1; toFlockWith4[2] = 2; toFlockWith4[3] = 4;

		flockParams->toFlockWith = toFlockWith1;
		flockParams2->toFlockWith = toFlockWith2;
		flockParams3->toFlockWith = toFlockWith3;
		flockParams4->toFlockWith = toFlockWith4;
				
		pf->init(*crowd->getCrowdQuery(), ts.getNavMesh(), crowd->getAgents(), crowd->getNbMaxAgents());

		crowd->getAgent(indexAgent2)->behavior = crowd->getAgent(indexAgent1)->behavior;
		crowd->getAgent(indexAgent3)->behavior = crowd->getAgent(indexAgent1)->behavior;
		crowd->getAgent(indexAgent4)->behavior = crowd->getAgent(indexAgent1)->behavior;

		crowd->getAgent(indexLeader)->behavior = pf;

		// Set the destination
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destLeader, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
																&pfParams->targetRef, 0);
		REQUIRE(pfParams->targetRef != 0);
		REQUIRE(pf->requestMoveTarget(indexLeader, pfParams->targetRef, destLeader));

		// We perform several update on the flocking group.
		for (int i = 0; i < 100; ++i)
			crowd->update(0.1f);

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
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent2)->npos) > flockParams->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent3)->npos) > flockParams->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent4)->npos) > flockParams->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent4)->npos, crowd->getAgent(indexAgent2)->npos) > flockParams->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent4)->npos, crowd->getAgent(indexAgent3)->npos) > flockParams->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(indexAgent3)->npos, crowd->getAgent(indexAgent2)->npos) > flockParams->separationDistance / 2.f);

		dtFree(toFlockWith1);
		dtFree(toFlockWith2);
		dtFree(toFlockWith3);
		dtFree(toFlockWith4);

		dtFlockingBehavior::free(flocking);
		dtPathFollowing::free(pf);
	}

	SECTION("Separation Behavior", "With the separation behavior, an agent tries to flee its target")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {2, 0, 0};
		float destAgt2[] = {2, 0, 0};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1);
		int indexAgent2 = crowd->addAgent(posAgt2);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);

		ts.defaultInitializeAgent(*crowd, indexAgent1);
		ts.defaultInitializeAgent(*crowd, indexAgent2);
		
		crowd->getAgent(indexAgent1)->collisionQueryRange = 0.f;

		crowd->update(2.0, 0);

		float agt1NewVel[3];
		float nilVector[] = {0, 0, 0};

		dtVcopy(agt1NewVel, crowd->getAgent(indexAgent1)->vel);	

		// Since the distance is set to 0, the velocity of the agent should be nil
		CHECK(dtVequal(agt1NewVel, nilVector));

		// We now set the distance
		crowd->getAgent(indexAgent1)->collisionQueryRange = 5.f;

		crowd->update(2.0, 0);

		dtVcopy(agt1NewVel, crowd->getAgent(indexAgent1)->vel);	

		// However the agent has not moved yet since no target was specified
		CHECK(dtVequal(agt1NewVel, nilVector));

		// Set the behavior
		dtSeparationBehavior* separation = dtSeparationBehavior::allocate(5);
		dtSeparationBehaviorParams* params = separation->addBehaviorParams(*crowd->getAgent(indexAgent1));
		params->separationAgents= crowd->getAgents();
		params->separationTargets = &indexAgent2;
		params->separationNbTargets = 1;
		params->separationWeight = 1.f;
		params->separationDistance = 2.f;
		
		crowd->getAgent(indexAgent1)->behavior = separation;

		crowd->update(2.0, 0);

		// The agent should now be moving to the left (away from the target)
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent2)->npos) > 2.1);

		// We perform several updates
		for (int i = 0; i < 100; ++i)
			crowd->update(1.0, 0);

		// The agent should have moved to the left far enough to respect the minimal distance
		CHECK(dtVdist2D(crowd->getAgent(indexAgent1)->npos, crowd->getAgent(indexAgent2)->npos) >= params->separationDistance);

		dtSeparationBehavior::free(separation);
	}

	SECTION("PathFollowing Behavior", "Tests whether the agent can follow a path to reach its destination")
	{
		float posAgt1[] = {0, 0, 0};
		float destAgt1[] = {-18, 0, 0};

		float posAgt2[] = {0, 0, 1};
		float destAgt2[] = {18, 0, 1};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1);
		int indexAgent2 = crowd->addAgent(posAgt2);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);

		ts.defaultInitializeAgent(*crowd, indexAgent1);
		ts.defaultInitializeAgent(*crowd, indexAgent2);

		dtPathFollowing* pf1 = dtPathFollowing::allocate(2);
		dtPathFollowingParams* pfParams = pf1->addBehaviorParams(*crowd->getAgent(indexAgent1));
		dtPathFollowingParams* pfParams2 = pf1->addBehaviorParams(*crowd->getAgent(indexAgent2));

		crowd->getAgent(indexAgent1)->behavior = pf1;
		crowd->getAgent(indexAgent2)->behavior = pf1;

		dtPolyRef poly1, poly2;
		float pos1NavMesh[3], pos2NavMesh[3];
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly1, pos1NavMesh);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly2, pos2NavMesh);
		pfParams->corridor.reset(poly1, pos1NavMesh);
		pfParams2->corridor.reset(poly2, pos2NavMesh);

		pf1->init(*crowd->getCrowdQuery(), ts.getNavMesh(), crowd->getAgents(), crowd->getNbMaxAgents());
		
		// Set the destinations
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&pfParams->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&pfParams2->targetRef, 0);
		REQUIRE(pf1->requestMoveTarget(indexAgent1, pfParams->targetRef, destAgt1));
		REQUIRE(pf1->requestMoveTarget(indexAgent2, pfParams2->targetRef, destAgt2));

		for (int i = 0; i < 100; ++i)
			crowd->update(1.0);

		float newPos[3];
		float newPos2[3];
		dtVcopy(newPos, crowd->getAgent(indexAgent1)->npos);
		dtVcopy(newPos2, crowd->getAgent(indexAgent2)->npos);

		CHECK(newPos[0] < -10.f);
		CHECK(newPos2[0] > 10.f);
	}

	SECTION("Alignment Behavior", "With the alignment behavior, an agent will align its velocity to its target")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {0, 0, 1};
		float posAgt3[] = {0, 0, -1};
		float destAgt2[] = {15, 0, 1};
		float destAgt3[] = {15, 0, -1};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1);
		int indexAgent2 = crowd->addAgent(posAgt2);
		int indexAgent3 = crowd->addAgent(posAgt3);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);
		REQUIRE_FALSE(indexAgent3 == -1);

		ts.defaultInitializeAgent(*crowd, indexAgent1);
		ts.defaultInitializeAgent(*crowd, indexAgent2);
		ts.defaultInitializeAgent(*crowd, indexAgent3);
		
		dtPathFollowing* pf1 = dtPathFollowing::allocate(2);
		dtPathFollowingParams* pfParams = pf1->addBehaviorParams(*crowd->getAgent(indexAgent2));
		dtPathFollowingParams* pfParams2 = pf1->addBehaviorParams(*crowd->getAgent(indexAgent3));

		dtPolyRef poly1, poly2;
		float pos1NavMesh[3], pos2NavMesh[3];
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly1, pos1NavMesh);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly2, pos2NavMesh);
		pfParams->corridor.reset(poly1, pos1NavMesh);
		pfParams2->corridor.reset(poly2, pos2NavMesh);

		pf1->init(*crowd->getCrowdQuery(), ts.getNavMesh(), crowd->getAgents(), crowd->getNbMaxAgents());
		
		crowd->getAgent(indexAgent2)->behavior = pf1;
		crowd->getAgent(indexAgent3)->behavior = pf1;

		// Set the destination
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &pfParams->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt3, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &pfParams2->targetRef, 0);

		REQUIRE(pf1->requestMoveTarget(indexAgent2, pfParams->targetRef, destAgt2));
		REQUIRE(pf1->requestMoveTarget(indexAgent3, pfParams2->targetRef, destAgt3));

		int targets[] = {1, 2};
		dtAlignmentBehavior* align = dtAlignmentBehavior::allocate(1);
		dtAlignmentBehaviorParams* params = align->addBehaviorParams(*crowd->getAgent(indexAgent1));
		params->alignmentAgents = crowd->getAgents();
		params->alignmentTargets = targets;
		params->alignmentNbTargets = 2;
		
		crowd->getAgent(indexAgent1)->behavior = align;

		for (int i = 0; i < 10; ++i)
			crowd->update(0.1f, 0);

		// The velocity should not be nil
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) > 0);
		// But not as fast as the others agents'
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) < dtVlen(crowd->getAgent(indexAgent3)->vel));


		for (int i = 0; i < 50; ++i)
			crowd->update(0.1f, 0);

		// After a while, the agent should almost be at max speed
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) >= 1.8f);


		for (int i = 0; i < 100; ++i)
			crowd->update(0.1f, 0);

		// After even longer, the agent should have stopped
		CHECK(dtVlen(crowd->getAgent(indexAgent1)->vel) <= 0.2f);

		dtAlignmentBehavior::free(align);
		dtPathFollowing::free(pf1);
	}

	SECTION("Cohesion Behavior", "With the cohesion behavior, an agent will be attracted to its targets")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {-5, 0, 1};
		float posAgt3[] = {-5, 0, -1};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1);
		int indexAgent2 = crowd->addAgent(posAgt2);
		int indexAgent3 = crowd->addAgent(posAgt3);

		REQUIRE_FALSE(indexAgent1 == -1);
		REQUIRE_FALSE(indexAgent2 == -1);
		REQUIRE_FALSE(indexAgent3 == -1);

		ts.defaultInitializeAgent(*crowd, indexAgent1);
		ts.defaultInitializeAgent(*crowd, indexAgent2);
		ts.defaultInitializeAgent(*crowd, indexAgent3);

		int targets[] = {1, 2};
		dtCohesionBehavior* cohesion = dtCohesionBehavior::allocate(5);
		dtCohesionAgentsParams* params = cohesion->addBehaviorParams(*crowd->getAgent(0));
		params->cohesionAgents = crowd->getAgents();
		params->cohesionTargets = targets;
		params->cohesionNbTargets = 2;
		
		crowd->getAgent(indexAgent1)->behavior = cohesion;

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

		dtCohesionBehavior::free(cohesion);
	}

	SECTION("GoTo Behavior", "With the goto behavior, an agent must move towards the given position")
	{
		dtGoToBehavior* go = dtGoToBehavior::allocate(5);
		dtGoToBehaviorParams* params = go->addBehaviorParams(*crowd->getAgent(0));

		float posAgt1[] = {0, 0, 0};
		float destAgt1[] = {15, 0, 0};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1);
		ts.defaultInitializeAgent(*crowd, indexAgent1);

		REQUIRE_FALSE(indexAgent1 == -1);

		params->gotoTarget = destAgt1;
		crowd->getAgent(indexAgent1)->behavior = go;

		crowd->update(2.0, 0);

		// The agent should move to the right
		CHECK(posAgt1[0] < crowd->getAgent(indexAgent1)->npos[0]);

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.01f, 0);

		// The agent should have arrived to its target
		CHECK(crowd->getAgent(indexAgent1)->npos[0] < 15.1f);
		CHECK(crowd->getAgent(indexAgent1)->npos[0] > 14.9f);

		params->gotoTarget[0] = 25.f;

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.1f, 0);

		// The agent should have reached the boundaries of the navigation mesh
		CHECK(crowd->getAgent(indexAgent1)->npos[0] < 20.1f);
		CHECK(crowd->getAgent(indexAgent1)->npos[0] > 19.6f);

		dtGoToBehavior::free(go);
	}
}

#endif
