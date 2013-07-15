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

#ifndef DETOURSEEKBEHAVIOR_H
#define DETOURSEEKBEHAVIOR_H

#include "DetourSteeringBehavior.h"

struct dtCrowdAgent;


/// Parameters for the seek behavior
/// @ingroup behavior
struct dtSeekBehaviorParams
{
	unsigned targetID;			///< The id of the agent we seek.
	float distance;				///< Minimal distance to keep between the agent and its target.
	float predictionFactor;		///< Used by the agent to predict the next position of the target. The higher the value, The better the prediction. 
								///  Nonetheless a big value is not realistic when agents are close to each other.
};

/// This class implements the seek behavior.
///
/// An agent with this behavior will try to reach its target.
/// It is possible to set the minimal distance between the agent and the target.
/// (the agent will never get any closer than this distance).
/// Also the agent can have a prediction factor, which means that it will predict the next
/// position of its target (according to its velocity) and head to it.
/// @ingroup behavior
class dtSeekBehavior : public dtSteeringBehavior<dtSeekBehaviorParams>
{
public:
	/// Creates an instance of the behavior
	///
	/// @param[in]	maxAgents	Estimation of the maximum number of agents using this behavior
	dtSeekBehavior(unsigned maxAgents);
	~dtSeekBehavior();

	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	///
	/// @return		A pointer on a newly allocated behavior
	static dtSeekBehavior* allocate(unsigned nbMaxAgents);

	/// Frees the given behavior
	///
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtSeekBehavior* ptr);

	virtual void computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
							  const dtSeekBehaviorParams& currentParams, dtSeekBehaviorParams& newParams);

private:
	virtual void applyForce(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float* force, float dt);
};


#endif // DETOURSEEKBEHAVIOR_H
