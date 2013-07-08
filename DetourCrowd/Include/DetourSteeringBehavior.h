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

#ifndef DETOURSTEERINGBEHAVIOR_H
#define DETOURSTEERINGBEHAVIOR_H

#include "DetourBehavior.h"
#include "DetourParametrizedBehavior.h"

#include "DetourCommon.h"
#include "DetourCrowd.h"

class dtCrowdQuery;


/// Interface defining a steering behavior.
/// @ingroup behavior
template <typename T = NoData>
class dtSteeringBehavior : public dtParametrizedBehavior<T>
{
public:
	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgent		Estimation of the maximum number of agents using this behavior
	explicit dtSteeringBehavior(unsigned nbMaxAgent);
	virtual ~dtSteeringBehavior();
	
	/// Computes the force that should be applied to the velocity of the given agent.
	///
	/// @param[in]	query	Allows the user to query data from the crowd.
	/// @param[in]	ag		The agent we want to update.
	/// @param[out]	force	The computed force.
	virtual void computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
							  const T& currentParams, T& newParams) = 0;

protected:
	/// Applies the previously computed force the velocity of the old agent and stores the result into the new agent.
	///
	/// @param[in]	query		Allows the user to query data from the crowd.
	/// @param[in]	oldAgent	The agent we want to update.
	/// @param[out]	newAgent	The agent storing the updated version of the oldAgent.
	/// @param[in]	force		The computed force.
	/// @param[in]	dt			The time, in seconds, to update the simulation. [Limit: > 0]
	virtual void applyForce(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float* force, float dt);

	virtual void doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
		const T& currentParams, T& newParams, float dt);
};

template<typename T>
dtSteeringBehavior<T>::dtSteeringBehavior(unsigned nbMaxAgent)
	: dtParametrizedBehavior<T>(nbMaxAgent)
{
}

template<typename T>
dtSteeringBehavior<T>::~dtSteeringBehavior()
{
}

template<typename T>
void dtSteeringBehavior<T>::applyForce(const dtCrowdQuery& /*query*/, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float* force, float dt)
{
	float acceleration[3];
	dtVcopy(acceleration, force);
	dtVclamp(acceleration, dtVlen(acceleration), oldAgent.maxAcceleration);

	dtVmad(newAgent.desiredVelocity, oldAgent.velocity, acceleration, dt);

	// Nil velocity
	if (dtVlen(newAgent.desiredVelocity) < EPSILON)
		return;

	dtVclamp(newAgent.desiredVelocity, dtVlen(newAgent.desiredVelocity), newAgent.maxSpeed);
}

template <typename T>
void dtSteeringBehavior<T>::doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
									 const T& currentParams, T& newParams, float dt)
{
	float desiredForce[] = {0, 0, 0};

	computeForce(query, oldAgent, desiredForce, currentParams, newParams);
	applyForce(query, oldAgent, newAgent, desiredForce, dt);
}

#endif
