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

		dtCrowdAgent ag1, ag2;

		dtSeekBehavior* seek = dtSeekBehavior::allocate(5);
		dtSeekBehaviorParams* params = seek->getBehaviorParams(crowd->getAgent(0)->id);
		params->targetID = 1;
		params->seekDistance = 0;
		params->seekPredictionFactor = 0;

		// Adding the agents to the crowd
		REQUIRE(crowd->addAgent(ag1, posAgt1));
		REQUIRE(crowd->addAgent(ag2, posAgt2));

		ts.defaultInitializeAgent(*crowd, ag1.id);
		ts.defaultInitializeAgent(*crowd, ag2.id);


		crowd->setAgentBehavior(ag1.id, seek);
				
		crowd->update(2.0, 0);

		float agt1NewPos[3];

		dtVcopy(agt1NewPos, crowd->getAgent(ag1.id)->position);	

		// The first agent should move to the right (since he is seeking the second agent)
		CHECK(posAgt1[0] < crowd->getAgent(ag2.id)->position[0]);

		dtSeekBehavior::free(seek);
		crowd->setAgentBehavior(ag1.id, 0);
	}

	SECTION("Flocking Behavior", "Unit tests about the flocking behavior")
	{
		dtPathFollowing* pf = dtPathFollowing::allocate(1);
		dtFlockingBehavior* flocking = dtFlockingBehavior::allocate(5, 2, 1, 1, 1);
		dtCrowdAgent ag1, ag2, ag3, ag4, ag5;

		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {1, 0, 1};
		float posAgt3[] = {1.5f, 0, 1};
		float posAgt4[] = {1, 0, 1.5f};
		float posLeader[] = {-0.5f, 0, -0.5f};
		float destLeader[] = {-14, 0, -14};

		// Adding the agents to the crowd
		REQUIRE(crowd->addAgent(ag1, posAgt1));
		REQUIRE(crowd->addAgent(ag2, posAgt2));
		REQUIRE(crowd->addAgent(ag3, posAgt3));
		REQUIRE(crowd->addAgent(ag4, posAgt4));
		REQUIRE(crowd->addAgent(ag5, posLeader));

		dtPathFollowingParams* pfParams = pf->getBehaviorParams(crowd->getAgent(4)->id);
		pfParams->init(256);

		dtFlockingBehaviorParams* flockParams = flocking->getBehaviorParams(crowd->getAgent(0)->id);
		dtFlockingBehaviorParams* flockParams2 = flocking->getBehaviorParams(crowd->getAgent(1)->id);
		dtFlockingBehaviorParams* flockParams3 = flocking->getBehaviorParams(crowd->getAgent(2)->id);
		dtFlockingBehaviorParams* flockParams4 = flocking->getBehaviorParams(crowd->getAgent(3)->id);

		dtPolyRef polyLeader;
		float posLeaderNavMesh[3];
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posLeader, crowd->getCrowdQuery()->getQueryExtents(), 
			crowd->getCrowdQuery()->getQueryFilter(), &polyLeader, posLeaderNavMesh);
		pfParams->corridor.reset(polyLeader, posLeaderNavMesh);
		
		ts.defaultInitializeAgent(*crowd, ag1.id);
		ts.defaultInitializeAgent(*crowd, ag2.id);
		ts.defaultInitializeAgent(*crowd, ag3.id);
		ts.defaultInitializeAgent(*crowd, ag4.id);
		ts.defaultInitializeAgent(*crowd, ag5.id);

		crowd->setAgentBehavior(ag1.id, flocking);
		crowd->setAgentBehavior(ag2.id, flocking);
		crowd->setAgentBehavior(ag3.id, flocking);
		crowd->setAgentBehavior(ag4.id, flocking);
		
		flocking->separationDistance = 2;
		flockParams->nbflockingTargets = 4;
		flockParams2->nbflockingTargets = 4;
		flockParams3->nbflockingTargets = 4;
		flockParams4->nbflockingTargets = 4;

		unsigned* toFlockWith1 = (unsigned*) dtAlloc(sizeof(unsigned) * flockParams->nbflockingTargets, DT_ALLOC_TEMP);
		unsigned* toFlockWith2 = (unsigned*) dtAlloc(sizeof(unsigned) * flockParams2->nbflockingTargets, DT_ALLOC_TEMP);
		unsigned* toFlockWith3 = (unsigned*) dtAlloc(sizeof(unsigned) * flockParams3->nbflockingTargets, DT_ALLOC_TEMP);
		unsigned* toFlockWith4 = (unsigned*) dtAlloc(sizeof(unsigned) * flockParams4->nbflockingTargets, DT_ALLOC_TEMP);

		toFlockWith1[0] = 1; toFlockWith1[1] = 2; toFlockWith1[2] = 3; toFlockWith1[3] = 4;
		toFlockWith2[0] = 0; toFlockWith2[1] = 2; toFlockWith2[2] = 3; toFlockWith2[3] = 4;
		toFlockWith3[0] = 0; toFlockWith3[1] = 1; toFlockWith3[2] = 3; toFlockWith3[3] = 4;
		toFlockWith4[0] = 0; toFlockWith4[1] = 1; toFlockWith4[2] = 2; toFlockWith4[3] = 4;

		flockParams->toFlockWith = toFlockWith1;
		flockParams2->toFlockWith = toFlockWith2;
		flockParams3->toFlockWith = toFlockWith3;
		flockParams4->toFlockWith = toFlockWith4;
				
		pf->init(*crowd->getCrowdQuery());

		crowd->setAgentBehavior(ag2.id, crowd->getAgent(ag1.id)->behavior);
		crowd->setAgentBehavior(ag3.id, crowd->getAgent(ag1.id)->behavior);
		crowd->setAgentBehavior(ag4.id, crowd->getAgent(ag1.id)->behavior);
		crowd->setAgentBehavior(ag5.id, pf);

		// Set the destination
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destLeader, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
																&pfParams->targetRef, 0);
		REQUIRE(pfParams->targetRef != 0);
		REQUIRE(pf->requestMoveTarget(ag5.id, pfParams->targetRef, destLeader));

		// We perform several update on the flocking group.
		for (int i = 0; i < 100; ++i)
			crowd->update(0.1f);

		// Each agent should be heading toward the upper left corner
		for (int i = 0; i < 4; ++i)
		{
			float vel[3];
			dtVcopy(vel, crowd->getAgent(i)->velocity);

			CHECK((vel[0] < 0 && vel[2] < 0));
		}

		// We perform several update in order to reach the point where the agents aren't moving anymore
		for (int i = 0; i < 1000; ++i)
			crowd->update(0.1f, 0);

		// We check that the minimal space between each agent is respected
		CHECK(dtVdist2D(crowd->getAgent(ag1.id)->position, crowd->getAgent(ag2.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(ag1.id)->position, crowd->getAgent(ag3.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(ag1.id)->position, crowd->getAgent(ag4.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(ag4.id)->position, crowd->getAgent(ag2.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(ag4.id)->position, crowd->getAgent(ag3.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd->getAgent(ag3.id)->position, crowd->getAgent(ag2.id)->position) > flocking->separationDistance / 2.f);

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
		dtCrowdAgent ag1, ag2;

		// Adding the agents to the crowd
		REQUIRE(crowd->addAgent(ag1, posAgt1));
		REQUIRE(crowd->addAgent(ag2, posAgt2));

		ts.defaultInitializeAgent(*crowd, ag1.id);
		ts.defaultInitializeAgent(*crowd, ag2.id);
		
		dtCrowdAgent ag;
		crowd->fetchAgent(ag, ag1.id);
		ag.collisionQueryRange = 0.f;
		crowd->applyAgent(ag);

		crowd->update(2.0, 0);

		float agt1NewVel[3];
		float nilVector[] = {0, 0, 0};

		dtVcopy(agt1NewVel, crowd->getAgent(ag1.id)->velocity);	

		// Since the distance is set to 0, the velocity of the agent should be nil
		CHECK(dtVequal(agt1NewVel, nilVector));

		// We now set the distance
		crowd->fetchAgent(ag, ag1.id);
		ag.collisionQueryRange = 5.f;
		crowd->applyAgent(ag);

		crowd->update(2.0, 0);

		dtVcopy(agt1NewVel, crowd->getAgent(ag1.id)->velocity);	

		// However the agent has not moved yet since no target was specified
		CHECK(dtVequal(agt1NewVel, nilVector));

		// Set the behavior
		dtSeparationBehavior* separation = dtSeparationBehavior::allocate(5);
		dtSeparationBehaviorParams* params = separation->getBehaviorParams(crowd->getAgent(ag1.id)->id);
		params->targetsID = &ag2.id;
		params->nbTargets = 1;
		params->separationWeight = 1.f;
		params->separationDistance = 2.f;
		
		crowd->setAgentBehavior(ag1.id, separation);

		crowd->update(2.0, 0);

		// The agent should now be moving to the left (away from the target)
		CHECK(dtVdist2D(crowd->getAgent(ag1.id)->position, crowd->getAgent(ag2.id)->position) > 2.1);

		// We perform several updates
		for (int i = 0; i < 100; ++i)
			crowd->update(1.0, 0);

		// The agent should have moved to the left far enough to respect the minimal distance
		CHECK(dtVdist2D(crowd->getAgent(ag1.id)->position, crowd->getAgent(ag2.id)->position) >= params->separationDistance);

		dtSeparationBehavior::free(separation);
	}

	SECTION("PathFollowing Behavior", "Tests whether the agent can follow a path to reach its destination")
	{
		float posAgt1[] = {0, 0, 0};
		float destAgt1[] = {-18, 0, 0};

		float posAgt2[] = {0, 0, 1};
		float destAgt2[] = {18, 0, 1};

		dtCrowdAgent ag1, ag2;

		// Adding the agents to the crowd
		REQUIRE(crowd->addAgent(ag1, posAgt1));
		REQUIRE(crowd->addAgent(ag2, posAgt2));

		ts.defaultInitializeAgent(*crowd, ag1.id);
		ts.defaultInitializeAgent(*crowd, ag2.id);

		dtPathFollowing* pf1 = dtPathFollowing::allocate(2);
		dtPathFollowingParams* pfParams = pf1->getBehaviorParams(crowd->getAgent(ag1.id)->id);
		dtPathFollowingParams* pfParams2 = pf1->getBehaviorParams(crowd->getAgent(ag2.id)->id);
		pfParams->init(256);
		pfParams2->init(256);

		crowd->setAgentBehavior(ag1.id, pf1);
		crowd->setAgentBehavior(ag2.id, pf1);

		dtPolyRef poly1, poly2;
		float pos1NavMesh[3], pos2NavMesh[3];
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly1, pos1NavMesh);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly2, pos2NavMesh);
		pfParams->corridor.reset(poly1, pos1NavMesh);
		pfParams2->corridor.reset(poly2, pos2NavMesh);

		pf1->init(*crowd->getCrowdQuery());
		
		// Set the destinations
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&pfParams->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), 
			&pfParams2->targetRef, 0);
		REQUIRE(pf1->requestMoveTarget(ag1.id, pfParams->targetRef, destAgt1));
		REQUIRE(pf1->requestMoveTarget(ag2.id, pfParams2->targetRef, destAgt2));

		for (int i = 0; i < 100; ++i)
			crowd->update(1.0);

		float newPos[3];
		float newPos2[3];
		dtVcopy(newPos, crowd->getAgent(ag1.id)->position);
		dtVcopy(newPos2, crowd->getAgent(ag2.id)->position);

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
		dtCrowdAgent ag1, ag2, ag3;

		// Adding the agents to the crowd
		REQUIRE(crowd->addAgent(ag1, posAgt1));
		REQUIRE(crowd->addAgent(ag2, posAgt2));
		REQUIRE(crowd->addAgent(ag3, posAgt3));

		ts.defaultInitializeAgent(*crowd, ag1.id);
		ts.defaultInitializeAgent(*crowd, ag2.id);
		ts.defaultInitializeAgent(*crowd, ag3.id);
		
		dtPathFollowing* pf1 = dtPathFollowing::allocate(2);
		dtPathFollowingParams* pfParams = pf1->getBehaviorParams(crowd->getAgent(ag2.id)->id);
		dtPathFollowingParams* pfParams2 = pf1->getBehaviorParams(crowd->getAgent(ag3.id)->id);
		pfParams->init(256);
		pfParams2->init(256);

		dtPolyRef poly1, poly2;
		float pos1NavMesh[3], pos2NavMesh[3];
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly1, pos1NavMesh);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &poly2, pos2NavMesh);
		pfParams->corridor.reset(poly1, pos1NavMesh);
		pfParams2->corridor.reset(poly2, pos2NavMesh);

		pf1->init(*crowd->getCrowdQuery());
		
		crowd->setAgentBehavior(ag2.id, pf1);
		crowd->setAgentBehavior(ag3.id, pf1);

		// Set the destination
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt2, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &pfParams->targetRef, 0);
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt3, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &pfParams2->targetRef, 0);

		REQUIRE(pf1->requestMoveTarget(ag2.id, pfParams->targetRef, destAgt2));
		REQUIRE(pf1->requestMoveTarget(ag3.id, pfParams2->targetRef, destAgt3));

		unsigned targets[] = {1, 2};
		dtAlignmentBehavior* align = dtAlignmentBehavior::allocate(1);
		dtAlignmentBehaviorParams* params = align->getBehaviorParams(crowd->getAgent(ag1.id)->id);
		params->alignmentTargets = targets;
		params->alignmentNbTargets = 2;
		
		crowd->setAgentBehavior(ag1.id, align);

		for (int i = 0; i < 10; ++i)
			crowd->update(0.1f, 0);

		// The velocity should not be nil
		CHECK(dtVlen(crowd->getAgent(ag1.id)->velocity) > 0);
		// But not as fast as the others agents'
		CHECK(dtVlen(crowd->getAgent(ag1.id)->velocity) < dtVlen(crowd->getAgent(ag3.id)->velocity));


		for (int i = 0; i < 50; ++i)
			crowd->update(0.1f, 0);

		// After a while, the agent should almost be at max speed
		CHECK(dtVlen(crowd->getAgent(ag1.id)->velocity) >= 1.8f);


		for (int i = 0; i < 100; ++i)
			crowd->update(0.1f, 0);

		// After even longer, the agent should have stopped
		CHECK(dtVlen(crowd->getAgent(ag1.id)->velocity) <= 0.2f);

		dtAlignmentBehavior::free(align);
		dtPathFollowing::free(pf1);
	}

	SECTION("Cohesion Behavior", "With the cohesion behavior, an agent will be attracted to its targets")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {-5, 0, 1};
		float posAgt3[] = {-5, 0, -1};
		dtCrowdAgent ag1, ag2, ag3;

		// Adding the agents to the crowd
		REQUIRE(crowd->addAgent(ag1, posAgt1));
		REQUIRE(crowd->addAgent(ag2, posAgt2));
		REQUIRE(crowd->addAgent(ag3, posAgt3));

		ts.defaultInitializeAgent(*crowd, ag1.id);
		ts.defaultInitializeAgent(*crowd, ag2.id);
		ts.defaultInitializeAgent(*crowd, ag3.id);

		unsigned targets[] = {1, 2};
		dtCohesionBehavior* cohesion = dtCohesionBehavior::allocate(5);
		dtCohesionBehaviorParams* params = cohesion->getBehaviorParams(crowd->getAgent(0)->id);
		params->cohesionTargets = targets;
		params->cohesionNbTargets = 2;
		
		crowd->setAgentBehavior(ag1.id, cohesion);

		for (int i = 0; i < 10; ++i)
			crowd->update(0.1f, 0);

		// The agent should be moving towards the targets
		CHECK(crowd->getAgent(ag1.id)->position[0] < 0);

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.1f, 0);

		dtCohesionBehavior::free(cohesion);
	}

	SECTION("GoTo Behavior", "With the goto behavior, an agent must move towards the given position")
	{
		dtArriveBehavior* go = dtArriveBehavior::allocate(5);
		dtArriveBehaviorParams* params = go->getBehaviorParams(crowd->getAgent(0)->id);

		float posAgt1[] = {0, 0, 0};
		float destAgt1[] = {15, 0, 0};
		dtCrowdAgent ag1;

		// Adding the agents to the crowd
		REQUIRE(crowd->addAgent(ag1, posAgt1));

		ts.defaultInitializeAgent(*crowd, ag1.id);

		params->target = destAgt1;

		crowd->setAgentBehavior(ag1.id, go);

		crowd->update(2.0, 0);

		// The agent should move to the right
		CHECK(posAgt1[0] < crowd->getAgent(ag1.id)->position[0]);

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.01f, 0);

		// The agent should have arrived to its target
		CHECK(crowd->getAgent(ag1.id)->position[0] < 15.1f);
		CHECK(crowd->getAgent(ag1.id)->position[0] > 14.9f);
		CHECK(crowd->getAgent(ag1.id)->velocity[0] < 0.0001f);

		params->target[0] = 25.f;

		for (int i = 0; i < 1000; ++i)
			crowd->update(0.1f, 0);

		// The agent should have reached the boundaries of the navigation mesh
		CHECK(crowd->getAgent(ag1.id)->position[0] < 20.1f);
		CHECK(crowd->getAgent(ag1.id)->position[0] > 19.6f);

		dtArriveBehavior::free(go);
	}
}

#endif
