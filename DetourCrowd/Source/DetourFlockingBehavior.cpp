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


dtFlockingBehavior::dtFlockingBehavior(unsigned nbMaxAgents, float separationWeight, float cohesionWeight, float alignmentWeight, float separationDistance)
	: dtSteeringBehavior<dtFlockingBehaviorParams>(nbMaxAgents)
    , separationWeight(separationWeight)
    , cohesionWeight(cohesionWeight)
    , alignmentWeight(alignmentWeight)
    , separationDistance(separationDistance)
	, m_separationBehavior(0)
    , m_cohesionBehavior(0)
    , m_alignmentBehavior(0)
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

dtFlockingBehavior* dtFlockingBehavior::allocate(unsigned nbMaxAgents, float separationWeight, float cohesionWeight, float alignmentWeight, float separationDistance)
{
	void* mem = dtAlloc(sizeof(dtFlockingBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtFlockingBehavior(nbMaxAgents, separationWeight, cohesionWeight, alignmentWeight, separationDistance);

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

void dtFlockingBehavior::computeForce(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, float* force, 
									  const dtFlockingBehaviorParams& currentParams, dtFlockingBehaviorParams& /*newParam*/)
{
	unsigned* neighborsList = currentParams.toFlockWith;
	unsigned nbNeighbors = currentParams.nbflockingTargets;

	const dtCrowdAgent** agents = (const dtCrowdAgent**) dtAlloc(sizeof(dtCrowdAgent*) * nbNeighbors, DT_ALLOC_TEMP);
	query.getAgents(neighborsList, nbNeighbors, agents);

	if (!agents || nbNeighbors == 0 || !neighborsList || !m_separationBehavior)
	{
		dtFree(agents);
		return;
	}

	dtSeparationBehaviorParams* separationParams;	
	dtCohesionBehaviorParams* cohesionParams;
	dtAlignmentBehaviorParams* alignmentParams;

	separationParams = m_separationBehavior->getBehaviorParams(oldAgent.id);
	cohesionParams = m_cohesionBehavior->getBehaviorParams(oldAgent.id);
	alignmentParams = m_alignmentBehavior->getBehaviorParams(oldAgent.id);

	if (!separationParams)
		separationParams = m_separationBehavior->getBehaviorParams(oldAgent.id);
	if (!cohesionParams)
		cohesionParams = m_cohesionBehavior->getBehaviorParams(oldAgent.id);
	if (!alignmentParams)
		alignmentParams = m_alignmentBehavior->getBehaviorParams(oldAgent.id);

	if (!separationParams || !cohesionParams || !alignmentParams)
	{
		dtFree(agents);
		return;
	}

	separationParams->targetsID = neighborsList;
	separationParams->nbTargets = nbNeighbors;
	separationParams->distance = this->separationDistance;
	separationParams->weight = separationWeight;

	cohesionParams->targets = neighborsList;
	cohesionParams->nbTargets = nbNeighbors;

	alignmentParams->targets = neighborsList;
	alignmentParams->nbTargets = nbNeighbors;

	float separationForce[] = {0, 0, 0};
	float cohesionForce[] = {0, 0, 0};
	float alignmentForce[] = {0, 0, 0};

	m_separationBehavior->computeForce(query, oldAgent, separationForce, *separationParams, *separationParams);
	m_cohesionBehavior->computeForce(query, oldAgent, cohesionForce, *cohesionParams, *cohesionParams);
	m_alignmentBehavior->computeForce(query, oldAgent, alignmentForce, *alignmentParams, *alignmentParams);

	dtVmad(force, force, separationForce, separationWeight);
	dtVmad(force, force, cohesionForce, cohesionWeight);
	dtVmad(force, force, alignmentForce, alignmentWeight);

	dtFree(agents);

	force[1] = 0;
}
