//
// Copyright (c) 2013 MASA Group jeremy.chanut@masagroup.net
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


static const float EPSILON = 0.0001f;

dtSeekBehavior::dtSeekBehavior(unsigned maxAgents)
	: dtSteeringBehavior()
{
}

dtSeekBehavior::~dtSeekBehavior()
{
}

void dtSeekBehavior::computeForce(const dtCrowdAgent* ag, float* force)
{
	const dtCrowdAgent* target = m_agentParams[ag->id]->targetID;
	const float predictionFactor = m_agentParams[ag->id]->seekPredictionFactor;

	// Required force in order to reach the target
	dtVsub(force, target->npos, ag->npos);

	// We take into account the prediction factor
	float scaledVelocity[3] = {0, 0, 0};

	dtVscale(scaledVelocity, target->vel, predictionFactor);
	dtVadd(force, force, scaledVelocity);

	// Set the force according to the maximum acceleration
	dtVclamp(force, dtVlen(force), ag->params.maxAcceleration);

	force[1] = 0;
}

void dtSeekBehavior::applyForce(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float* force, float dt)
{
	float tmpForce[] = {0, 0, 0};
	float newVelocity[] = {0, 0, 0};
	const dtCrowdAgent* target = m_agentParams[oldAgent->id]->targetID;
	const float distance = m_agentParams[oldAgent->id]->seekDistance;

	// Adapting the force to the dt and the previous velocity
	dtVscale(tmpForce, force, dt);
	dtVadd(newVelocity, oldAgent->vel, tmpForce);

	float currentSpeed = dtVlen(oldAgent->vel);
	// Required distance to reach nil speed according to the acceleration and current speed
	float slowDist = currentSpeed * (currentSpeed - 0) / oldAgent->params.maxAcceleration;
	float distToObj = dtVdist(oldAgent->npos, target->npos) - oldAgent->params.radius - target->params.radius - distance;

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
		newAgent->desiredSpeed = oldAgent->params.maxSpeed;

	// Check for maximal speed
	dtVclamp(newVelocity, dtVlen(newVelocity), oldAgent->params.maxSpeed);

	dtVcopy(newAgent->dvel, newVelocity);
}
