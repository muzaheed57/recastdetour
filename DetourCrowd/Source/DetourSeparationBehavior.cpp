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

#include "DetourSeparationBehavior.h"

#include "DetourCommon.h"
#include "DetourCrowd.h"

#include <cmath>
#include <new>


dtSeparationBehavior::dtSeparationBehavior(unsigned nbMaxAgents)
	: dtSteeringBehavior<dtSeparationBehaviorParams>(nbMaxAgents)
{
}

dtSeparationBehavior::~dtSeparationBehavior()
{
}

dtSeparationBehavior* dtSeparationBehavior::allocate(unsigned nbMaxAgents)
{
	void* mem = dtAlloc(sizeof(dtSeparationBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtSeparationBehavior(nbMaxAgents);

	return 0;
}

void dtSeparationBehavior::free(dtSeparationBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtSeparationBehavior();
	dtFree(ptr);
	ptr = 0;
}

void dtSeparationBehavior::update(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	if (!oldAgent || !newAgent)
		return;

	float force[] = {0, 0, 0};

	computeForce(oldAgent, force);
	applyForce(oldAgent, newAgent, force, dt);
}

void dtSeparationBehavior::computeForce(const dtCrowdAgent* ag, float* force)
{
	const dtCrowdAgent* agents = getBehaviorParams(*ag)->separationAgents;
	const int* targets = getBehaviorParams(*ag)->separationTargets;
	const int nbTargets = getBehaviorParams(*ag)->separationNbTargets;
	const float distance = getBehaviorParams(*ag)->separationDistance;

	if (!agents || !targets || nbTargets <= 0)
		return;

	float maxDistance = (distance < 0.f) ? ag->collisionQueryRange : distance;
	const float invSeparationDist = 1.f / maxDistance;
	float weight;
	int count = 0;

	for (int i = 0; i < nbTargets; ++i)
	{
		const dtCrowdAgent& target = agents[targets[i]];

		if (!target.active)
			continue;

		float diff[3];
		dtVsub(diff, ag->npos, target.npos);

		float dist = dtVlen(diff) - ag->radius - target.radius;

		if (dist > maxDistance || dist < EPSILON)
			continue;

		weight = getBehaviorParams(*ag)->separationWeight * (1.f - dtSqr(dist * invSeparationDist));

		++count;
		dtVnormalize(diff);
		dtVmad(force, force, diff, weight / dist);
	}

	if (count > 0)
		dtVscale(force, force, (1.f / (float) count));
}
