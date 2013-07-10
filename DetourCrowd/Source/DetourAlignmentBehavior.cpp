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


static const float EPSILON = 0.0001f;

dtAlignmentBehavior::dtAlignmentBehavior()
	: m_agents(0)
{
}

void dtAlignmentBehavior::update(dtCrowdAgent* ag, float* force)
{
	if (!ag || !m_agents || !m_targets || !m_nbTargets)
		return;

	int count = 0;

	for (int i = 0; i < m_nbTargets; ++i)
		if (m_agents[m_targets[i]].active)
		{
			++ count;
			dtVadd(force, force, m_agents[m_targets[i]].vel);
		}

	dtVscale(force, force, 1.f / (float) count);

	dtVsub(force, force, ag->vel);
}

void dtAlignmentBehavior::update(dtCrowdAgent* ag, float dt)
{
	if (!ag || !m_agents || !m_targets || !m_nbTargets)
		return;

	float force[] = {0, 0, 0};

	update(ag, force);
	applyVelocity(ag, force, dt);
}

void dtAlignmentBehavior::setTargets(const int* targetsIndices, int nbTargets)
{
	m_targets = targetsIndices;
	m_nbTargets = nbTargets;
}

void dtAlignmentBehavior::applyVelocity(dtCrowdAgent* ag, float* velocity, float dt)
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