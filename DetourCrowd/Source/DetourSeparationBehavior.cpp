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


static const float EPSILON = 0.0001f;

dtSeparationBehavior::dtSeparationBehavior()
	: dtSteeringBehavior()
{
}

dtSeparationBehavior::~dtSeparationBehavior()
{
}

void dtSeparationBehavior::update(dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	if (!oldAgent || !newAgent)
		return;

	float force[] = {0, 0, 0};

	computeForce(oldAgent, force);
	applyForce(oldAgent, newAgent, force, dt);
}

void dtSeparationBehavior::computeForce(const dtCrowdAgent* ag, float* force)
{
	const dtCrowdAgent* agents = ag->params.separationAgents;
	const int* targets = ag->params.separationTargets;
	const int nbTargets = ag->params.separationNbTargets;
	const float distance = ag->params.separationDistance;

	if (!agents || !targets || nbTargets <= 0)
		return;

	float maxDistance = (distance < 0.f) ? ag->params.collisionQueryRange : distance;
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

		float dist = dtVlen(diff) - ag->params.radius - target.params.radius;

		if (dist > maxDistance || dist < EPSILON)
			continue;

		weight = ag->params.separationWeight * (1.f - dtSqr(dist * invSeparationDist));

		++count;
		dtVnormalize(diff);
		dtVmad(force, force, diff, weight / dist);
	}

	if (count > 0)
		dtVscale(force, force, (1.f / (float) count));
}
