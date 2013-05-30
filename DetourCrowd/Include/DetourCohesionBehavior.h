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

#ifndef DETOURCOHESIONBEHAVIOR_H
#define DETOURCOHESIONBEHAVIOR_H

#include "DetourSteeringBehavior.h"

struct dtCrowdAgent;
class dtGoToBehavior;


struct dtCohesionAgentsParams
{
	const dtCrowd* crowd;					///< The crowd used to access agents
	const int* cohesionTargets;				///< The indices of the targets
	int cohesionNbTargets;					///< The number of target
};


/// Defines the cohesion behavior.
///
/// An agent using the cohesion behavior will move towards the 
/// center of gravity of its targets.
class dtCohesionBehavior : public dtSteeringBehavior<dtCohesionAgentsParams>
{
public:
	dtCohesionBehavior(unsigned nbMaxAgents);
	~dtCohesionBehavior();

	static dtCohesionBehavior* allocate(unsigned nbMaxAgents);
	static void free(dtCohesionBehavior* ptr);

	virtual void update(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt);
	virtual void computeForce(const dtCrowdAgent* ag, float* force);

private:
	/// Computes the average position of the given agents.
	///
	/// @param[in]	agents		The agents of the crowd.
	/// @param[in]	targets		The indices of the targets.
	/// @param[in]	nbTargets	The number of targets.
	/// @param[out]	center		The computed center of gravity.
	void getGravityCenter(const dtCrowdAgent* agents, const int* targets, int nbTargets, float* center);

	dtGoToBehavior* m_gotoBehabior;								///< The GoTo behavior used to move the agent.
};

#endif
