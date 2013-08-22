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

#include "DetourAlignmentBehavior.h"

#include "DetourCrowd.h"
#include "DetourCommon.h"

#include <new>


dtAlignmentBehavior::dtAlignmentBehavior(unsigned nbMaxAgents)
	: dtSteeringBehavior<dtAlignmentBehaviorParams>(nbMaxAgents)
{
}

dtAlignmentBehavior::~dtAlignmentBehavior()
{
}

void dtAlignmentBehavior::computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
									   const dtAlignmentBehaviorParams& currentParams, dtAlignmentBehaviorParams& /*newParams*/)
{
	const unsigned* targets = currentParams.targets;
	const unsigned nbTargets = currentParams.nbTargets;

	const dtCrowdAgent** agents = (const dtCrowdAgent**) dtAlloc(sizeof(dtCrowdAgent*) * nbTargets, DT_ALLOC_TEMP);
	query.getAgents(targets, nbTargets, agents);

	if (!agents || !targets || !nbTargets)
	{
		dtFree(agents);
		return;
	}

	int count = 0;

	for (unsigned i = 0; i < nbTargets; ++i)
		if (agents[i]->active)
		{
			++ count;
			dtVadd(force, force, agents[i]->velocity);
		}

	dtVscale(force, force, 1.f / (float) count);
	dtVsub(force, force, ag.velocity);

	dtFree(agents);

	force[1] = 0;
}

dtAlignmentBehavior* dtAlignmentBehavior::allocate(unsigned nbMaxAgents)
{
	void* mem = dtAlloc(sizeof(dtAlignmentBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtAlignmentBehavior(nbMaxAgents);

	return 0;
}

void dtAlignmentBehavior::free(dtAlignmentBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtAlignmentBehavior();
	dtFree(ptr);
	ptr = 0;
}