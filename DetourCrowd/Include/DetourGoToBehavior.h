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

#ifndef DETOURGOTOBEHAVIOR_H
#define DETOURGOTOBEHAVIOR_H

#include "DetourSteeringBehavior.h"

struct dtCrowdAgent;


/// Parameters for the Arrive behavior
/// @ingroup behavior
struct dtArriveBehaviorParams
{
	float distance;		///< Minimal distance to keep between the agent and its target.
	float* target;		///< The position we want to reach.
};

/// Defines the GoTo behavior.
///
/// This behavior allows the user to set a target (a position [x, y, z]), and the agent will try to reach it.
/// @ingroup behavior
class dtArriveBehavior : public dtSteeringBehavior<dtArriveBehaviorParams>
{
public:
	dtArriveBehavior(unsigned nbMaxAgents);
	~dtArriveBehavior();

	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	///
	/// @return		A pointer on a newly allocated behavior
	static dtArriveBehavior* allocate(unsigned nbMaxAgents);

	/// Frees the given behavior
	///
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtArriveBehavior* ptr);

	virtual void computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
							  const dtArriveBehaviorParams& currentParams, dtArriveBehaviorParams& newParams);

private:
	virtual void applyForce(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float* force, float dt);
};

#endif