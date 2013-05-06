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

#ifndef DETOURPIPELINETEST_H
#define DETOURPIPELINETEST_H

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
#include "DetourPathFollowing.h"
#include "DetourPipelineBehavior.h"
#include "DetourSeekBehavior.h"
#include "DetourSeparationBehavior.h"


TEST_CASE("DetourCrowdTest/Pipeline", "Tests about the pipeline behavior")
{
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

	SECTION("Adding and Removing behaviors to the pipeline", "Trying to add and remove behaviors into the pipeline, should not crash")
	{
		dtPipelineBehavior* pipeline = dtAllocBehavior<dtPipelineBehavior>();
		param1.steeringBehavior = pipeline;

		float posAgt1[] = {0, 0.2f, 0};
		float destAgt1[] = {15, 0, 0};

		// Adding the agents to the crowd
		int indexAgent1 = crowd->addAgent(posAgt1, &param1);
		REQUIRE_FALSE(indexAgent1 == -1);

		// Set the destination
		dtPolyRef dest;
		crowd->getNavMeshQuery()->findNearestPoly(destAgt1, crowd->getQueryExtents(), crowd->getFilter(), &dest, 0);
		REQUIRE(crowd->requestMoveTarget(indexAgent1, dest, destAgt1));

		crowd->update(0.5, 0);

		float agt1NewPos[3];
		dtVcopy(agt1NewPos, crowd->getAgent(indexAgent1)->npos);	

		// Since no behavior is affected to the pipeline, the agent must not have moved
		CHECK(dtVequal(agt1NewPos, posAgt1));

		dtPathFollowing* pf = dtAllocBehavior<dtPathFollowing>();
		REQUIRE(pf->init(crowd->getPathQueue(), crowd->getNavMeshQuery(), crowd->getQueryExtents(), crowd->getEditableFilter(), crowd->getMaxPathResult(), 
				crowd->getAgents(), crowd->getNbMaxAgents(), crowd->getAnims()));

		pipeline->setBehaviors(0, 1);
		param1.steeringBehavior = pipeline;

		crowd->updateAgentParameters(indexAgent1, &param1);

		// Still no behavior was given (null pointer), so nothing should have moved.
		crowd->update(0.5, 0);

		dtVcopy(agt1NewPos, crowd->getAgent(indexAgent1)->npos);	

		// Since no behavior is affected to the pipeline, the agent must not have moved
		CHECK(dtVequal(agt1NewPos, posAgt1));

		dtBehavior* behavior = pf;

		// This time we affect the right behavior but with a size of 0
		pipeline->setBehaviors(&behavior, 0);
		param1.steeringBehavior = pipeline;

		crowd->updateAgentParameters(indexAgent1, &param1);
		crowd->update(0.5, 0);

		// The agent should not have moved
		dtVcopy(agt1NewPos, crowd->getAgent(indexAgent1)->npos);	
		CHECK(dtVequal(agt1NewPos, posAgt1));

		// This time we affect the behavior with the right size
		pipeline->setBehaviors(&behavior, 1);
		param1.steeringBehavior = pipeline;

		crowd->updateAgentParameters(indexAgent1, &param1);
		crowd->update(0.5, 0);

		dtVcopy(agt1NewPos, crowd->getAgent(indexAgent1)->npos);	

		// A behavior has been affected to the pipeline, the agent should have moved
		CHECK(!dtVequal(agt1NewPos, posAgt1));

		dtFreeBehavior<dtPathFollowing>(pf);
		dtFreeBehavior<dtPipelineBehavior>(pipeline);
	}
}

#endif