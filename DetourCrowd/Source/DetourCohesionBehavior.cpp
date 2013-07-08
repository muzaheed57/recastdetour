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
	: dtSteeringBehavior<dtCohesionBehaviorParams>(nbMaxAgents)
{
}

dtCohesionBehavior::~dtCohesionBehavior()
{
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

void dtCohesionBehavior::computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
									  const dtCohesionBehaviorParams& currentParams, dtCohesionBehaviorParams& /*newParams*/)
{
	const unsigned* targets = currentParams.targets;
	const unsigned nbTargets = currentParams.nbTargets;

	float center[] = {0, 0, 0};

	if (nbTargets == 0 || !targets)
		return;

	getGravityCenter(query, targets, nbTargets, center);
	dtVsub(force, center, ag.position);

	force[1] = 0;
}

void dtCohesionBehavior::getGravityCenter(const dtCrowdQuery& query, const unsigned* targets, int nbTargets, float* center)
{
	int count = 0;

	for (int i = 0; i < nbTargets; ++i)
	{
		const dtCrowdAgent& target = *query.getAgent(targets[i]);

		if (!target.active)
			continue;

		dtVadd(center, center, target.position);
		++count;
	}

	dtVscale(center, center, 1.f / (float) count);
}
