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
	: dtSteeringBehavior()
	, m_separationBehavior(0)
	, m_cohesionBehavior(0)
	, m_alignmentBehavior(0)
	, m_separationWeight(0)
	, m_cohesionWeight(0)
	, m_alignmentWeight(0)
{
	m_separationBehavior = dtAllocBehavior<dtSeparationBehavior>();
	m_cohesionBehavior = dtAllocBehavior<dtCohesionBehavior>();
	m_alignmentBehavior = dtAllocBehavior<dtAlignmentBehavior>();
}

dtFlockingBehavior::dtFlockingBehavior(float separationWeight, float cohesionWeight, float alignmentWeight)
	: dtSteeringBehavior()
	, m_separationBehavior(0)
	, m_cohesionBehavior(0)
	, m_alignmentBehavior(0)
	, m_separationWeight(separationWeight)
	, m_cohesionWeight(cohesionWeight)
	, m_alignmentWeight(alignmentWeight)
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

void dtFlockingBehavior::update(dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	if (!oldAgent || !newAgent)
		return;

	dtCrowdAgent* agents = oldAgent->params.flockingAgents;
	int* neighborsList = oldAgent->params.toFlockWith;
	int nbNeighbors = oldAgent->params.nbFlockingNeighbors;
	
	if (!agents || nbNeighbors == 0 || !neighborsList || !m_separationBehavior)
		return;

	float totalForce[] = {0, 0, 0};

	oldAgent->params.separationAgents = agents;
	oldAgent->params.separationTargets = neighborsList;
	oldAgent->params.separationNbTargets = nbNeighbors;
	oldAgent->params.separationDistance = oldAgent->params.flockingSeparationDistance;
	oldAgent->params.separationWeight = m_separationWeight;
	
	oldAgent->params.cohesionAgents = agents;
	oldAgent->params.cohesionTargets = neighborsList;
	oldAgent->params.cohesionNbTargets = nbNeighbors;

	oldAgent->params.alignmentAgents = agents;
	oldAgent->params.alignmentTargets = neighborsList;
	oldAgent->params.alignmentNbTargets = nbNeighbors;
	
	computeForce(oldAgent, totalForce);
	applyForce(oldAgent, newAgent, totalForce, dt);
}

void dtFlockingBehavior::computeForce(const dtCrowdAgent* ag, float* force)
{
	float separationForce[] = {0, 0, 0};
	float cohesionForce[] = {0, 0, 0};
	float alignmentForce[] = {0, 0, 0};

	m_separationBehavior->computeForce(ag, separationForce);
	m_cohesionBehavior->computeForce(ag, cohesionForce);
	m_alignmentBehavior->computeForce(ag, alignmentForce);

	dtVmad(force, force, separationForce, m_separationWeight);
	dtVmad(force, force, cohesionForce, m_cohesionWeight);
	dtVmad(force, force, alignmentForce, m_alignmentWeight);
}
