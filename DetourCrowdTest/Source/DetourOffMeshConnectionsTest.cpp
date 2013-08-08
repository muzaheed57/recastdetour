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


SCENARIO("DetourCrowdTest/OffMeshConnections", "[OffMesh] Check if the agents know when they are walking on offMesh connections or using them")
{
	GIVEN("An offMesh connection at (0, 0, 0) with a radius of 1")
	{
		TestScene ts;

		OffMeshConnectionCreator* offmeshCreator = ts.getOffMeshCreator();

		// Creation of the offMesh connection
		offmeshCreator->vert[0] = 0;
		offmeshCreator->vert[1] = 0.2;
		offmeshCreator->vert[2] = 0;
		offmeshCreator->vert[3] = 5;
		offmeshCreator->vert[4] = 0.2;
		offmeshCreator->vert[5] = 5;

		offmeshCreator->radius[0] = 1.f;
		offmeshCreator->bidir[0] = 1;
		offmeshCreator->areas[0] = SAMPLE_POLYAREA_JUMP;
		offmeshCreator->flags[0] = SAMPLE_POLYFLAGS_JUMP;
		offmeshCreator->ids[0] = 0;
		offmeshCreator->count++;

		dtCrowd* crowd = ts.createSquareScene(2, 0.5f);
		float posAgt1[] = {0, 0, 0};
		dtCrowdAgent ag;

		REQUIRE(crowd != 0);

		// Adding the agent to the crowd
		REQUIRE(crowd->addAgent(ag, posAgt1));
		REQUIRE(ts.defaultInitializeAgent(*crowd, ag.id));
		crowd->fetchAgent(ag, ag.id);
		
		WHEN("An agent is placed at (0, 0, 0")
		{
			THEN("The agent detects the offMesh connection it is on")
			{
				CHECK(crowd->getCrowdQuery()->getOffMeshConnection(ag.id) != 0);
			}			
		}
		WHEN("The agent is moved to (-2, 0, 0)")
		{
			float pos2[] = {-2, 0, 0};
			CHECK(crowd->updateAgentPosition(ag.id, pos2));

			THEN("It doesn't detect the offMesh connection anymore")
			{
				CHECK(crowd->getCrowdQuery()->getOffMeshConnection(ag.id) == 0);
			}
		}
		WHEN("We ask the same to check for a radius of 1.1 around him")
		{
			THEN("It detects the offMesh connection again")
			{
				CHECK(crowd->getCrowdQuery()->getOffMeshConnection(ag.id, 1.1f) != 0);
			}
		}
		WHEN("We try to detect an offMesh connection with wrong parameters")
		{
			THEN("We don't find it and it does not crash")
			{
				CHECK(crowd->getCrowdQuery()->getOffMeshConnection(9999) == 0);
				CHECK(crowd->getCrowdQuery()->getOffMeshConnection(ag.id, -5.f) == 0);
			}
		}
		WHEN("The agent passes over the connection")
		{
			float pos[] = {-0.2, 0, -0.2};

			CHECK(crowd->updateAgentPosition(ag.id, pos));
			CHECK(crowd->getCrowdQuery()->getOffMeshConnection(ag.id) != 0);

			crowd->fetchAgent(ag, ag.id);
			crowd->getCrowdQuery()->startOffMeshConnection(ag, 
				*crowd->getCrowdQuery()->getOffMeshConnection(ag.id));
			crowd->applyAgent(ag);

			THEN("It moves toward the other side of the connection")
			{
				crowd->updatePosition(0.1f);

				crowd->fetchAgent(ag, ag.id);
				CHECK(ag.position[0] > pos[0]);
				CHECK(ag.position[2] > pos[2]);
			}

			THEN("After several updates it reaches the other side")
			{
				for (unsigned i = 0; i < 20; ++i)
					crowd->updatePosition(0.1f);

				crowd->fetchAgent(ag, ag.id);
				CHECK(ag.position[0] > 4.5f);
				CHECK(ag.position[2] > 4.5f);
			}
		}
	}
}