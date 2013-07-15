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

#include "DetourGoToBehavior.h"

#include "DetourCommon.h"
#include "DetourCrowd.h"

#include <new>

dtArriveBehavior::dtArriveBehavior(unsigned nbMaxAgents)
	: dtSteeringBehavior<dtArriveBehaviorParams>(nbMaxAgents)
{
}

dtArriveBehavior::~dtArriveBehavior()
{
}

dtArriveBehavior* dtArriveBehavior::allocate(unsigned nbMaxAgents)
{
	void* mem = dtAlloc(sizeof(dtArriveBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtArriveBehavior(nbMaxAgents);

	return 0;
}

void dtArriveBehavior::free(dtArriveBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtArriveBehavior();
	dtFree(ptr);
	ptr = 0;
}

void dtArriveBehavior::computeForce(const dtCrowdQuery& /*query*/, const dtCrowdAgent& ag, float* force, 
									const dtArriveBehaviorParams& currentParams, dtArriveBehaviorParams& /*newParams*/)
{
	const float* target = currentParams.target;

	// Required velocity in order to reach the target
	dtVsub(force, target, ag.position);
}

void dtArriveBehavior::applyForce(const dtCrowdQuery& /*query*/, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float* force, float dt)
{
	const float* target = getBehaviorParams(oldAgent.id)->target;
	const float distance = getBehaviorParams(oldAgent.id)->distance;

	float tmpForce[3] = {0, 0, 0};
	float newVelocity[] = {0, 0, 0};

	// Set the velocity according to the maximum acceleration
	dtVnormalize(force);
	dtVscale(force, force, oldAgent.maxAcceleration);

	// Adapting the velocity to the dt and the previous velocity
	dtVscale(tmpForce, force, dt);
	dtVadd(newVelocity, oldAgent.velocity, tmpForce);

	float currentSpeed = dtVlen(oldAgent.velocity);
	// Required distance to reach nil speed according to the acceleration and current speed
	float slowDist = currentSpeed * (currentSpeed - 0) / oldAgent.maxAcceleration;
	float distToObj = dtVdist(oldAgent.position, target) - distance;

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
