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

#include "DetourPathFollowing.h"
#include "DetourCollisionAvoidance.h"

#include <vector>

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

TEST_CASE("DetourPipelineTest/Pipeline", "Tests about the pipeline behavior")
{
	TestScene ts;
	dtCrowd* crowd = ts.createSquareScene(20, 0.5);

	REQUIRE(crowd != 0);

	dtPipelineBehavior* pipeline = dtPipelineBehavior::allocate();

	float posAgt1[] = {0, 0.2f, 0};
	float destAgt1[] = {15, 0, 0};

	dtCrowdAgent ag;

	// Adding the agents to the crowd
	REQUIRE(crowd->addAgent(ag, posAgt1));
	ts.defaultInitializeAgent(*crowd, ag.id);

	crowd->setAgentBehavior(ag.id, pipeline);

	crowd->update(0.5, 0);

	float agt1NewPos[3];
	dtVcopy(agt1NewPos, crowd->getAgent(ag.id)->position);

	// Since no behavior is affected to the pipeline, the agent must not have moved
	CHECK(dtVequal(agt1NewPos, posAgt1));

	dtPathFollowing* pf = dtPathFollowing::allocate(5);
	dtPathFollowingParams* pfParams = pf->getBehaviorParams(crowd->getAgent(0)->id);
	pfParams->init(256);
	pfParams->preparePath(crowd->getAgent(0)->position, *crowd->getCrowdQuery());

	// Set the destination
	dtPolyRef dest;
	float nearest[3];
	crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt1, crowd->getCrowdQuery()->getQueryExtents(), crowd->getCrowdQuery()->getQueryFilter(), &dest, nearest);

	REQUIRE(dest != 0);		
	REQUIRE(pf->init(*crowd->getCrowdQuery()));
	REQUIRE(pf->requestMoveTarget(ag.id, dest, destAgt1));

	SECTION("Adding and Removing behaviors to the pipeline", "Trying to add and remove behaviors into the pipeline, should not crash")
	{
		CHECK(pipeline->setBehaviors(0, 1));

		// Still no behavior was given (null pointer), so nothing should have moved.
		crowd->update(0.5, 0);

		dtVcopy(agt1NewPos, crowd->getAgent(ag.id)->position);	

		// Since no behavior is affected to the pipeline, the agent must not have moved
		CHECK(dtVequal(agt1NewPos, posAgt1));

		dtBehavior* behavior = pf;

		// This time we affect the right behavior but with a size of 0
		CHECK(pipeline->setBehaviors(&behavior, 0));

		crowd->update(0.5, 0);

		// The agent should not have moved
		dtVcopy(agt1NewPos, crowd->getAgent(ag.id)->position);	
		CHECK(dtVequal(agt1NewPos, posAgt1));

		// This time we affect the behavior with the right size
		CHECK(pipeline->setBehaviors(&behavior, 1));

		crowd->update(0.5, 0);

		dtVcopy(agt1NewPos, crowd->getAgent(ag.id)->position);	

		// A behavior has been affected to the pipeline, the agent should have moved
		CHECK(!dtVequal(agt1NewPos, posAgt1));

	}

	SECTION("Using a container", "Using a container to store behaviors, set them to the pipeline and then destroy the container. The behaviors should still be in the pipeline")
	{
		dtCollisionAvoidance* ca = dtCollisionAvoidance::allocate(crowd->getAgentCount());
		ca->init();

		dtCollisionAvoidanceParams* params = ca->getBehaviorParams(ag.id);

		if (params)
		{
			params->debug = 0;
			params->velBias = 0.4f;
			params->weightDesVel = 2.0f;
			params->weightCurVel = 0.75f;
			params->weightSide = 0.75f;
			params->weightToi = 2.5f;
			params->horizTime = 2.5f;
			params->gridSize = 33;
			params->adaptiveDivs = 7;
			params->adaptiveRings = 2;
			params->adaptiveDepth = 5;
		}

		std::vector<dtBehavior*> behaviors;
		behaviors.push_back(pf);
		behaviors.push_back(ca);

		CHECK(pipeline->setBehaviors(behaviors.data(), behaviors.size()));

		behaviors.clear();

		dtVcopy(posAgt1, crowd->getAgent(ag.id)->position);	
		crowd->update(0.5);

		// The agent should have moved
		dtVcopy(agt1NewPos, crowd->getAgent(ag.id)->position);	
		CHECK(!dtVequal(agt1NewPos, posAgt1));

		dtCollisionAvoidance::free(ca);
	}

	dtPathFollowing::free(pf);
	dtPipelineBehavior::free(pipeline);
}
