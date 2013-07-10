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
	: m_targets(0)
	, m_nbTargets(0)
	, m_agents(0)
	, m_separationDistance(-1.f)
	, m_separationWeight(-1.f)
{
}

dtSeparationBehavior::~dtSeparationBehavior()
{
}

void dtSeparationBehavior::update(dtCrowdAgent* ag, float* velocity, float dt)
{
	// If there are no targets, then the agent doesn't move
	if (!m_agents || !m_targets || m_nbTargets <= 0)
		return;

	prepareVelocity(ag, velocity);
}

void dtSeparationBehavior::update(dtCrowdAgent* ag, float dt)
{
	// If there are no targets, then the agent doesn't move
	if (!m_agents || !m_targets || m_nbTargets <= 0)
		return;

	float velocity[] = {0, 0, 0};

	prepareVelocity(ag, velocity);
	applyVelocity(ag, velocity, dt);
}

void dtSeparationBehavior::setTargets(const int* targets, int nbTargets)
{
	m_targets = targets;
	m_nbTargets = nbTargets;
}

void dtSeparationBehavior::prepareVelocity(const dtCrowdAgent* ag, float* velocity)
{
	float maxDistance = (m_separationDistance < 0.f) ? ag->params.collisionQueryRange : m_separationDistance;
	const float invSeparationDist = 1.f / maxDistance;
	float weight;
	int count = 0;

	for (int i = 0; i < m_nbTargets; ++i)
	{
		dtCrowdAgent& target = m_agents[m_targets[i]];

		if (!target.active)
			continue;

		float diff[3];
		dtVsub(diff, ag->npos, target.npos);

		float dist = dtVlen(diff) - ag->params.radius - target.params.radius;

		if (dist > maxDistance || dist < EPSILON)
			continue;

		weight = m_separationWeight * (1.f - dtSqr(dist * invSeparationDist));

		++count;
		dtVnormalize(diff);
		dtVmad(velocity, velocity, diff, weight / dist);
	}

	if (count > 0)
		dtVscale(velocity, velocity, (1.f / (float) count));
}

void dtSeparationBehavior::applyVelocity(dtCrowdAgent* ag, float* velocity, float dt)
{
	dtVclamp(velocity, dtVlen(velocity), ag->params.maxAcceleration);

	dtVscale(velocity, velocity, dt);
	dtVadd(ag->dvel ,ag->vel, velocity);

	// Nil velocity
	if (dtVlen(ag->dvel) < EPSILON)
	{
		ag->desiredSpeed = 0.f;
		return;
	}

	dtVclamp(ag->dvel, dtVlen(ag->dvel), ag->params.maxSpeed);

	ag->desiredSpeed = dtVlen(ag->dvel);
}
