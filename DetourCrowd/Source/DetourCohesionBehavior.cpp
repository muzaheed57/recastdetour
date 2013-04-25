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

#include "DetourCohesionBehavior.h"

#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourGoToBehavior.h"

#include <new>


static const float EPSILON = 0.0001f;

dtCohesionBehavior::dtCohesionBehavior()
	: dtSteeringBehavior()
	, m_gotoBehabior(0)
{
	m_gotoBehabior = dtAllocBehavior<dtGoToBehavior>();
}

dtCohesionBehavior::~dtCohesionBehavior()
{
	dtFreeBehavior<dtGoToBehavior>(m_gotoBehabior);
}

void dtCohesionBehavior::update(dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	if (!oldAgent || !newAgent)
		return;
	
	float force[] = {0, 0, 0};
	float center[] = {0, 0, 0};
	const dtCrowdAgent* agents = oldAgent->params.cohesionAgents;
	const int* targets = oldAgent->params.cohesionTargets;
	const int nbTargets = oldAgent->params.cohesionNbTargets;

	getGravityCenter(agents, targets, nbTargets, center);

	oldAgent->params.gotoTarget = center;
	oldAgent->params.gotoDistance = 0.f;
	m_gotoBehabior->update(oldAgent, newAgent, dt);
}

void dtCohesionBehavior::computeForce(const dtCrowdAgent* ag, float* force)
{
	const dtCrowdAgent* agents = ag->params.cohesionAgents;
	const int* targets = ag->params.cohesionTargets;
	const int nbTargets = ag->params.cohesionNbTargets;
	int count = 0;
	float center[] = {0, 0, 0};

	getGravityCenter(agents, targets, nbTargets, center);

	dtVsub(force, center, ag->npos);
}

void dtCohesionBehavior::getGravityCenter(const dtCrowdAgent* agents, const int* targets, int nbTargets, float* center)
{
	int count = 0;

	for (int i = 0; i < nbTargets; ++i)
	{
		const dtCrowdAgent& target = agents[targets[i]];

		if (!target.active)
			continue;

		dtVadd(center, center, target.npos);
		++count;
	}

	dtVscale(center, center, 1.f / (float) count);
}
