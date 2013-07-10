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


static const float EPSILON = 0.0001f;

dtSeekBehavior::dtSeekBehavior()
	: m_target(0)
	, m_distance(1.f)
	, m_predictionFactor(0.f)
{
}

dtSeekBehavior::~dtSeekBehavior()
{
}

void dtSeekBehavior::update(dtCrowdAgent* ag, float* force, const float dt) const
{
	if (!m_target || !m_target->active)
		return;

	float desiredForce[3] = {0, 0, 0};
	float tmpForce[3] = {0, 0, 0};
	
	// Required velocity in order to reach the target
	dtVsub(desiredForce, m_target->npos, ag->npos);

	// We take into account the prediction
	float scaledVelocity[3] = {0, 0, 0};

	dtVscale(scaledVelocity, m_target->vel, m_predictionFactor);
	dtVadd(desiredForce, desiredForce, scaledVelocity);

	// Set the velocity according to the maximum acceleration
	dtVnormalize(desiredForce);
	dtVscale(desiredForce, desiredForce, ag->params.maxAcceleration);

	// Adapting the velocity to the dt and the previous velocity
	dtVscale(tmpForce, desiredForce, dt);
	dtVadd(force, ag->vel, tmpForce);
	
	float currentSpeed = dtVlen(ag->vel);
	// Required distance to reach nil speed according to the acceleration and current speed
	float slowDist = currentSpeed * (currentSpeed - 0) / ag->params.maxAcceleration;
	float distToObj = dtVdist(ag->npos, m_target->npos) - ag->params.radius - m_target->params.radius - m_distance;

	// If we have reached the target, we stop
	if (distToObj <= EPSILON)
	{
		dtVset(force, 0, 0, 0);
		ag->desiredSpeed = 0.f;
	}
	// If the have to slow down
	else if (distToObj < slowDist)
	{
		float slowDownRadius = distToObj / slowDist;
		dtVscale(force, force, slowDownRadius);
		ag->desiredSpeed = dtVlen(force);
	}
	// Else, we move as fast as possible
	else
		ag->desiredSpeed = ag->params.maxSpeed;
		
	// Check for maximal speed
	dtVclamp(force, dtVlen(force), ag->params.maxSpeed);
}
