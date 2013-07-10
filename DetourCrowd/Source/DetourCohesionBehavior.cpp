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
	: m_agents(0)
	, m_targets(0)
	, m_nbTargets(0)
	, m_gotoBehabior(0)
{
	m_gotoBehabior = dtAllocBehavior<dtGoToBehavior>();
}

dtCohesionBehavior::~dtCohesionBehavior()
{
	dtFreeBehavior<dtGoToBehavior>(m_gotoBehabior);
}

void dtCohesionBehavior::update(dtCrowdAgent* ag, float* force)
{
	if (!m_agents || !m_targets || !m_nbTargets)
		return;

	int count = 0;
	dtVset(force, 0, 0, 0);

	for (int i = 0; i < m_nbTargets; ++i)
	{
		dtCrowdAgent& target = m_agents[m_targets[i]];

		if (!target.active)
			continue;

		dtVadd(force, force, target.npos);
		++count;
	}

	dtVscale(force, force, 1.f / (float) count);
	dtVsub(force, force, ag->npos);
}

void dtCohesionBehavior::update(dtCrowdAgent* ag, float dt)
{
	if (!m_agents || !m_targets || !m_nbTargets)
		return;

	int count = 0;
	float force[] = {0, 0, 0};
	float center[] = {0, 0, 0};

	for (int i = 0; i < m_nbTargets; ++i)
	{
		dtCrowdAgent& target = m_agents[m_targets[i]];

		if (!target.active)
			continue;

		dtVadd(center, center, target.npos);
		++count;
	}

	dtVscale(center, center, 1.f / (float) count);

	m_gotoBehabior->m_target = center;
	m_gotoBehabior->m_distance = 0.f;
	m_gotoBehabior->update(ag, force, dt);

	dtVcopy(ag->dvel, force);
}

void dtCohesionBehavior::setTargets(const int* targets, int nbTargets)
{
	m_targets = targets;
	m_nbTargets = nbTargets;
}