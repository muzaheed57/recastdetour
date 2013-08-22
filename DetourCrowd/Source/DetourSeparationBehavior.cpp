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

void dtSeparationBehavior::computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
										const dtSeparationBehaviorParams& currentParams, dtSeparationBehaviorParams& /*newParams*/)
{
	const unsigned* targets = currentParams.targetsID;
	const unsigned nbTargets = currentParams.nbTargets;
	const float distance = currentParams.distance;

	const dtCrowdAgent** agents = (const dtCrowdAgent**) dtAlloc(sizeof(dtCrowdAgent*) * nbTargets, DT_ALLOC_TEMP);
	query.getAgents(targets, nbTargets, agents);

	if (!agents || !targets || nbTargets <= 0) {
		dtFree(agents);
		return;
	}

	const float invSeparationDist = 1.f / distance;
	float weight;
	int count = 0;

	for (unsigned i = 0; i < nbTargets; ++i)
	{
		const dtCrowdAgent& target = *agents[i];

		if (!target.active)
			continue;

		float diff[3];
		dtVsub(diff, ag.position, target.position);

		float dist = dtVlen(diff) - ag.radius - target.radius;

		if (dist > distance || dist < EPSILON)
			continue;

		weight = currentParams.weight * (1.f - dtSqr(dist * invSeparationDist));

		++count;
		dtVnormalize(diff);
		dtVmad(force, force, diff, weight / dist);
	}

	if (count > 0)
		dtVscale(force, force, (1.f / (float) count));

	dtFree(agents);
}

