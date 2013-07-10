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

struct dtCrowdAgent;
class dtGoToBehavior;


/// Defines the cohesion behavior.
///
/// An agent using the cohesion behavior will move towards the 
/// center of gravity of its targets.
class dtCohesionBehavior
{
public:
	dtCohesionBehavior();
	~dtCohesionBehavior();

	/// Updates the velocity of the given agent.
	///
	/// @param[in]	ag		The agent we want to update.
	/// @param[out]	force	The time, in seconds, to update the simulation. [Limit: > 0]
	void update(dtCrowdAgent* ag, float* force);

	/// Updates the velocity of the given agent.
	///
	/// @param[in]	ag	The agent we want to update.
	/// @param[in]	dt	The time, in seconds, to update the simulation. [Limit: > 0]
	void update(dtCrowdAgent* ag, float dt);

	void setAgents(dtCrowdAgent* agents) { m_agents = agents; }
	void setTargets(const int* targets, int nbTargets);

private:
	dtCrowdAgent* m_agents;			///< The list of agents the indices are refering to.
	const int* m_targets;			///< The indices of the targets
	int m_nbTargets;				///< The number of target
	dtGoToBehavior* m_gotoBehabior;	///< The GoTo behavior used to move the agent.
};

#endif DETOURCOHESIONBEHAVIOR_H