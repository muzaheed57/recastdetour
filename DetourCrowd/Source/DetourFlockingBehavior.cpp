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


dtFlockingBehavior::dtFlockingBehavior(unsigned nbMaxAgents, float separationWeight, float cohesionWeight, float alignmentWeight)
	: dtSteeringBehavior<dtFlockingBehaviorParams>(nbMaxAgents)
	, m_separationBehavior(0)
	, m_cohesionBehavior(0)
	, m_alignmentBehavior(0)
	, separationWeight(separationWeight)
	, cohesionWeight(cohesionWeight)
	, alignmentWeight(alignmentWeight)
{
	m_separationBehavior = dtSeparationBehavior::allocate(nbMaxAgents);
	m_cohesionBehavior = dtCohesionBehavior::allocate(nbMaxAgents);
	m_alignmentBehavior = dtAlignmentBehavior::allocate(nbMaxAgents);
}

dtFlockingBehavior::~dtFlockingBehavior()
{
	dtSeparationBehavior::free(m_separationBehavior);
	dtCohesionBehavior::free(m_cohesionBehavior);
	dtAlignmentBehavior::free(m_alignmentBehavior);
}

dtFlockingBehavior* dtFlockingBehavior::allocate(unsigned nbMaxAgents, float separationWeight, float cohesionWeight, float alignmentWeight)
{
	void* mem = dtAlloc(sizeof(dtFlockingBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtFlockingBehavior(nbMaxAgents, separationWeight, cohesionWeight, alignmentWeight);

	return 0;
}

void dtFlockingBehavior::free(dtFlockingBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtFlockingBehavior();
	dtFree(ptr);
	ptr = 0;
}

void dtFlockingBehavior::update(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	if (!oldAgent || !newAgent)
		return;

	dtFlockingBehaviorParams* params = getBehaviorParams(oldAgent->id);

	if (params == 0)
		return;

	int* neighborsList = params->toFlockWith;
	int nbNeighbors = params->nbflockingTargets;

	const dtCrowdAgent** agents = (const dtCrowdAgent**) dtAlloc(sizeof(dtCrowdAgent*) * nbNeighbors, DT_ALLOC_TEMP);
	params->crowd->getAgents(neighborsList, nbNeighbors, agents);
	
	if (!agents || nbNeighbors == 0 || !neighborsList || !m_separationBehavior)
	{
		dtFree(agents);
		return;
	}

	float totalForce[] = {0, 0, 0};

	dtSeparationBehaviorParams* separationParams;	
	dtCohesionAgentsParams* cohesionParams;
	dtAlignmentBehaviorParams* alignmentParams;

	separationParams = m_separationBehavior->addBehaviorParams(oldAgent->id);
	cohesionParams = m_cohesionBehavior->addBehaviorParams(oldAgent->id);
	alignmentParams = m_alignmentBehavior->addBehaviorParams(oldAgent->id);

	if (!separationParams)
		separationParams = m_separationBehavior->getBehaviorParams(oldAgent->id);
	if (!cohesionParams)
		cohesionParams = m_cohesionBehavior->getBehaviorParams(oldAgent->id);
	if (!alignmentParams)
		alignmentParams = m_alignmentBehavior->getBehaviorParams(oldAgent->id);

	if (!separationParams || !cohesionParams || !alignmentParams)
	{
		dtFree(agents);
		return;
	}

	separationParams->crowd = params->crowd;
	separationParams->separationTargets = neighborsList;
	separationParams->separationNbTargets = nbNeighbors;
	separationParams->separationDistance = params->separationDistance;
	separationParams->separationWeight = separationWeight;

	cohesionParams->crowd = params->crowd;
	cohesionParams->cohesionTargets = neighborsList;
	cohesionParams->cohesionNbTargets = nbNeighbors;

	alignmentParams->crowd = params->crowd;
	alignmentParams->alignmentTargets = neighborsList;
	alignmentParams->alignmentNbTargets = nbNeighbors;
	
	computeForce(oldAgent, totalForce);
	applyForce(oldAgent, newAgent, totalForce, dt);

	dtFree(agents);
}

void dtFlockingBehavior::computeForce(const dtCrowdAgent* ag, float* force)
{
	float separationForce[] = {0, 0, 0};
	float cohesionForce[] = {0, 0, 0};
	float alignmentForce[] = {0, 0, 0};

	m_separationBehavior->computeForce(ag, separationForce);
	m_cohesionBehavior->computeForce(ag, cohesionForce);
	m_alignmentBehavior->computeForce(ag, alignmentForce);

	dtVmad(force, force, separationForce, separationWeight);
	dtVmad(force, force, cohesionForce, cohesionWeight);
	dtVmad(force, force, alignmentForce, alignmentWeight);
}
