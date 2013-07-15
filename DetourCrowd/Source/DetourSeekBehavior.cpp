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

#include "DetourSeekBehavior.h"

#include "DetourCommon.h"
#include "DetourCrowd.h"

#include <new>


dtSeekBehavior::dtSeekBehavior(unsigned maxAgents)
	: dtSteeringBehavior<dtSeekBehaviorParams>(maxAgents)
{
}

dtSeekBehavior::~dtSeekBehavior()
{
}

dtSeekBehavior* dtSeekBehavior::allocate(unsigned nbMaxAgents)
{
	void* mem = dtAlloc(sizeof(dtSeekBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtSeekBehavior(nbMaxAgents);

	return 0;
}

void dtSeekBehavior::free(dtSeekBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtSeekBehavior();
	dtFree(ptr);
	ptr = 0;
}

void dtSeekBehavior::computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
								  const dtSeekBehaviorParams& currentParams, dtSeekBehaviorParams& /*newParams*/)
{
	const int targetID = currentParams.targetID;
	const dtCrowdAgent* target = query.getAgent(targetID);
	const float predictionFactor = currentParams.predictionFactor;

	if (!target || !target->active)
		return;

	// Required force in order to reach the target
	dtVsub(force, target->position, ag.position);

	// We take into account the prediction factor
	float scaledVelocity[3] = {0, 0, 0};

	dtVscale(scaledVelocity, target->velocity, predictionFactor);
	dtVadd(force, force, scaledVelocity);

	// Set the force according to the maximum acceleration
	dtVclamp(force, dtVlen(force), ag.maxAcceleration);

	force[1] = 0;
}

void dtSeekBehavior::applyForce(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float* force, float dt)
{
	float tmpForce[] = {0, 0, 0};
	float newVelocity[] = {0, 0, 0};
	const int targetID = getBehaviorParams(oldAgent.id)->targetID;
	const dtCrowdAgent* target = query.getAgent(targetID);
	const float distance = getBehaviorParams(oldAgent.id)->distance;

	// Adapting the force to the dt and the previous velocity
	dtVscale(tmpForce, force, dt);
	dtVadd(newVelocity, oldAgent.velocity, tmpForce);

	float currentSpeed = dtVlen(oldAgent.velocity);
	// Required distance to reach nil speed according to the acceleration and current speed
	float slowDist = currentSpeed * (currentSpeed - 0) / oldAgent.maxAcceleration;
	float distToObj = dtVdist(oldAgent.position, target->position) - oldAgent.radius - target->radius - distance;

	// If we have reached the target, we stop
	if (distToObj <= EPSILON)
	{
		dtVset(newVelocity, 0, 0, 0);
	}
	// If the have to slow down
	else if (distToObj < slowDist)
	{
		float slowDownRadius = distToObj / slowDist;
		dtVscale(newVelocity, newVelocity, slowDownRadius);
	}

	// Check for maximal speed
	dtVclamp(newVelocity, dtVlen(newVelocity), oldAgent.maxSpeed);

	dtVcopy(newAgent.desiredVelocity, newVelocity);
}
