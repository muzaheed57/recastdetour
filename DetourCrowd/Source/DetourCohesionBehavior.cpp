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


dtCohesionBehavior::dtCohesionBehavior(unsigned nbMaxAgents)
	: dtSteeringBehavior<dtCohesionAgentsParams>(nbMaxAgents)
	, m_gotoBehabior(0)
{
	m_gotoBehabior = dtGoToBehavior::allocate(nbMaxAgents);
}

dtCohesionBehavior::~dtCohesionBehavior()
{
	dtGoToBehavior::free(m_gotoBehabior);
	m_gotoBehabior = 0;
}

dtCohesionBehavior* dtCohesionBehavior::allocate(unsigned nbMaxAgents)
{
	void* mem = dtAlloc(sizeof(dtCohesionBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtCohesionBehavior(nbMaxAgents);

	return 0;
}

void dtCohesionBehavior::free(dtCohesionBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtCohesionBehavior();
	dtFree(ptr);
	ptr = 0;
}

void dtCohesionBehavior::update(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	if (!oldAgent || !newAgent)
		return;
	
	float center[] = {0, 0, 0};
	const dtCrowdAgent* agents = getBehaviorParams(*oldAgent)->cohesionAgents;
	const int* targets = getBehaviorParams(*oldAgent)->cohesionTargets;
	const int nbTargets = getBehaviorParams(*oldAgent)->cohesionNbTargets;

	getGravityCenter(agents, targets, nbTargets, center);

	dtGoToBehaviorParams* gotoParams;

	gotoParams = m_gotoBehabior->addBehaviorParams(*oldAgent);

	if (!gotoParams)
		gotoParams = m_gotoBehabior->getBehaviorParams(*oldAgent);

	if (!gotoParams)
		return;

	gotoParams->gotoTarget = center;
	gotoParams->gotoDistance = 0.f;

	m_gotoBehabior->update(oldAgent, newAgent, dt);
}

void dtCohesionBehavior::computeForce(const dtCrowdAgent* ag, float* force)
{
	const dtCrowdAgent* agents = getBehaviorParams(*ag)->cohesionAgents;
	const int* targets = getBehaviorParams(*ag)->cohesionTargets;
	const int nbTargets = getBehaviorParams(*ag)->cohesionNbTargets;
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
