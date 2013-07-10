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

#ifndef DETOURSEPARATIONBEHAVIOR_H
#define DETOURSEPARATIONBEHAVIOR_H

struct dtCrowdAgent;
class dtCrowd;


/// Implementation of the separation behavior.
///
/// An agent using this behavior will try to keep its distance from one or more targets.
/// The minimum distance to keep from the targets can be specified.
class dtSeparationBehavior
{
public:
	dtSeparationBehavior();
	~dtSeparationBehavior();

	/// Computes the new velocity of the agent
	///
	/// @param[in]	ag		The agent whose velocity must be updated.
	/// @param[in]	dt		The time, in seconds, since the last frame.
	void update(dtCrowdAgent* ag, float dt);

	/// Computes the new velocity of the agent
	///
	/// @param[in]	ag		The agent whose velocity must be updated.
	/// @param[out]	force	The computed velocity.
	/// @param[in]	dt		The time, in seconds, since the last frame.
	void update(dtCrowdAgent* ag, float* force, float dt);
	
	/// Sets the targets the agent must keep its distance from.
	///
	/// @param[in]	targets		The list of agent representing the targets.
	/// @param[in]	nbTargets	The number of targets to avoid.
	void setTargets(const int* targets, int nbTargets);

	void setAgents(dtCrowdAgent* agents) { m_agents = agents; }

	float m_separationDistance;	///< From distance from which the agent considers the targets that must be avoided.
	float m_separationWeight;	///< A coefficient defining how agressively the agent should avoid the targets.
	
private:
	/// Computes the new velocity for the agent.
	///
	/// @param[in]	ag				The agent whose velocity must be updated.
	/// @param[out]	velocity		The new velocity for the agent.
	void prepareVelocity(const dtCrowdAgent* ag, float* velocity);

	/// Scales the velocity according the agent's parameters.
	///
	/// @param[out]	ag				The agent whose velocity must be updated.
	/// @param[in]	velocity		The new velocity for the agent.
	/// @param[in]	dt				The time, in seconds, since the last frame.
	void applyVelocity(dtCrowdAgent* ag, float* velocity, float dt);

	const int* m_targets;		///< The others agents we want to keep our distances from.
	int m_nbTargets;			///< The number of targets.
	dtCrowdAgent* m_agents;		///< The list of agents the indices are refering to.
};

#endif // DETOURSEPARATIONBEHAVIOR_H