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

#include "DetourFlockingBehavior.h"

#include "DetourAlignmentBehavior.h"
#include "DetourAlloc.h"
#include "DetourCohesionBehavior.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourSeparationBehavior.h"

#include <cstring>
#include <new>


static const float EPSILON = 0.0001f;

dtFlockingBehavior::dtFlockingBehavior()
{
}

dtFlockingBehavior::dtFlockingBehavior(float desiredSeparation, 
									   float separationWeight, float cohesionWeight, float alignmentWeight, 
									   dtCrowdAgent* agents)
	: m_agents(agents)
	, m_separationDistance(desiredSeparation)
	, m_separationWeight(separationWeight)
	, m_cohesionWeight(cohesionWeight)
	, m_alignmentWeight(alignmentWeight)
	, m_separationBehavior(0)
	, m_cohesionBehavior(0)
	, m_alignmentBehavior(0)
{
	m_separationBehavior = dtAllocBehavior<dtSeparationBehavior>();
	m_cohesionBehavior = dtAllocBehavior<dtCohesionBehavior>();
	m_alignmentBehavior = dtAllocBehavior<dtAlignmentBehavior>();
}

dtFlockingBehavior::~dtFlockingBehavior()
{
	dtFreeBehavior<dtSeparationBehavior>(m_separationBehavior);
	dtFreeBehavior<dtCohesionBehavior>(m_cohesionBehavior);
	dtFreeBehavior<dtAlignmentBehavior>(m_alignmentBehavior);
}

void dtFlockingBehavior::update(dtCrowdAgent* ag, float dt)
{
	const int* neighborsList = ag->params.toFlockWith;
	int nbNeighbors = ag->params.nbFlockingNeighbors;
	
	if (nbNeighbors == 0 || !neighborsList || !m_separationBehavior)
		return;

	float separationForce[] = {0, 0, 0};
	float cohesionForce[] = {0, 0, 0};
	float alignmentForce[] = {0, 0, 0};
	float totalForce[] = {0, 0, 0};

	m_separationBehavior->setAgents(m_agents);
	m_separationBehavior->setTargets(neighborsList, nbNeighbors);
	m_separationBehavior->m_separationDistance = m_separationDistance;
	m_separationBehavior->m_separationWeight = m_separationWeight;
	m_separationBehavior->update(ag, separationForce, dt);
	
	m_cohesionBehavior->setAgents(m_agents);
	m_cohesionBehavior->setTargets(neighborsList, nbNeighbors);
	m_cohesionBehavior->update(ag, cohesionForce);

	m_alignmentBehavior->setAgents(m_agents);
	m_alignmentBehavior->setTargets(neighborsList, nbNeighbors);
	m_alignmentBehavior->update(ag, alignmentForce);
	
	dtVmad(totalForce, totalForce, separationForce, m_separationWeight);
	dtVmad(totalForce, totalForce, cohesionForce, m_cohesionWeight);
	dtVmad(totalForce, totalForce, alignmentForce, m_alignmentWeight);
	
	prepareVelocity(totalForce, ag, dt);
}

void dtFlockingBehavior::prepareVelocity(float* desiredVelocityDelta, dtCrowdAgent* ag, float dt)
{
	dtVclamp(desiredVelocityDelta, dtVlen(desiredVelocityDelta), ag->params.maxAcceleration);

	dtVscale(desiredVelocityDelta, desiredVelocityDelta, dt);
	dtVadd(ag->dvel ,ag->vel, desiredVelocityDelta);

	// Nil velocity
	if (dtVlen(ag->dvel) < EPSILON)
	{
		ag->desiredSpeed = 0.f;
		return;
	}
	
	dtVclamp(ag->dvel, dtVlen(ag->dvel), ag->params.maxSpeed);

	ag->desiredSpeed = dtVlen(ag->dvel);
}
