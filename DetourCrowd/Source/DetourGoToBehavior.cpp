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

dtGoToBehavior::dtGoToBehavior(unsigned nbMaxAgents)
	: dtSteeringBehavior<dtGoToBehaviorParams>(nbMaxAgents)
{
}

dtGoToBehavior::~dtGoToBehavior()
{
}

dtGoToBehavior* dtGoToBehavior::allocate(unsigned nbMaxAgents)
{
	void* mem = dtAlloc(sizeof(dtGoToBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtGoToBehavior(nbMaxAgents);

	return 0;
}

void dtGoToBehavior::free(dtGoToBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtGoToBehavior();
	dtFree(ptr);
	ptr = 0;
}

void dtGoToBehavior::update(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	if (!oldAgent || !newAgent)
		return;
	
	float desiredForce[3] = {0, 0, 0};
	
	computeForce(oldAgent, desiredForce);
	applyForce(oldAgent, newAgent, desiredForce, dt);
}

void dtGoToBehavior::computeForce(const dtCrowdAgent* ag, float* force)
{
	if (!ag)
		return;

	const float* target = getBehaviorParams(ag->id)->gotoTarget;

	// Required velocity in order to reach the target
	dtVsub(force, target, ag->npos);
}

void dtGoToBehavior::applyForce(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float* force, float dt)
{
	const float* target = getBehaviorParams(oldAgent->id)->gotoTarget;
	const float distance = getBehaviorParams(oldAgent->id)->gotoDistance;

	float tmpForce[3] = {0, 0, 0};
	float newVelocity[] = {0, 0, 0};

	// Set the velocity according to the maximum acceleration
	dtVnormalize(force);
	dtVscale(force, force, oldAgent->maxAcceleration);

	// Adapting the velocity to the dt and the previous velocity
	dtVscale(tmpForce, force, dt);
	dtVadd(newVelocity, oldAgent->vel, tmpForce);

	float currentSpeed = dtVlen(oldAgent->vel);
	// Required distance to reach nil speed according to the acceleration and current speed
	float slowDist = currentSpeed * (currentSpeed - 0) / oldAgent->maxAcceleration;
	float distToObj = dtVdist(oldAgent->npos, target) - distance;

	// If we have reached the target, we stop
	if (distToObj <= EPSILON)
	{
		dtVset(newVelocity, 0, 0, 0);
		newAgent->desiredSpeed = 0.f;
	}
	// If the have to slow down
	else if (distToObj < slowDist)
	{
		float slowDownRadius = distToObj / slowDist;
		dtVscale(newVelocity, newVelocity, slowDownRadius);
		newAgent->desiredSpeed = dtVlen(newVelocity);
	}
	// Else, we move as fast as possible
	else
		newAgent->desiredSpeed = oldAgent->maxSpeed;

	// Check for maximal speed
	dtVclamp(newVelocity, dtVlen(newVelocity), oldAgent->maxSpeed);

	dtVcopy(newAgent->dvel, newVelocity);
}
