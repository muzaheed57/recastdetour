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

#ifndef DETOURALIGNMENTBEHAVIOR_H
#define DETOURALIGNMENTBEHAVIOR_H

struct dtCrowdAgent;


/// Defines the alignment behavior.
///
/// An agent using this behavior will keep its velocity aligned with its targets'.
class dtAlignmentBehavior
{
public:
	dtAlignmentBehavior();

	/// Updates the velocity of the given agent.
	///
	/// @param[in]	ag	The agent we want to update.
	/// @param[in]	dt	The time, in seconds, to update the simulation. [Limit: > 0]
	void update(dtCrowdAgent* ag, float dt);

	/// Updates the velocity of the given agent.
	///
	/// @param[in]	ag		The agent we want to update.
	/// @param[out]	force	The time, in seconds, to update the simulation. [Limit: > 0]
	void update(dtCrowdAgent* ag, float* force);

	void setAgents(dtCrowdAgent* agents) { m_agents = agents; }
	void setTargets(const int* targetsIndices, int nbTargets);

private:
	/// Applies the previously computed velocity according the the given agent's parameters (acceleration, max speed, etc.).
	/// Called after the update() method.
	///
	/// @param[in]	velocity	The output velocity.
	/// @param[in]	ag			The agent whose velocity we want to compute.
	/// @param[in]	dt			The time, in seconds, to update the simulation. [Limit: > 0]
	void applyVelocity(dtCrowdAgent* ag, float* velocity, float dt);

	dtCrowdAgent* m_agents;	///< The list of agents the indices are refering to.
	const int* m_targets;	///< The indices of the targets
	int m_nbTargets;		///< The number of target
};

#endif DETOURALIGNMENTBEHAVIOR_H