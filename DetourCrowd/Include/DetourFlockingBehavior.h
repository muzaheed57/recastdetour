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

#ifndef DETOURFLOCKINGBEHAVIOR_H
#define DETOURFLOCKINGBEHAVIOR_H

struct dtCrowdAgent;
class dtSeparationBehavior;
class dtCohesionBehavior;
class dtAlignmentBehavior;


/// @defgroup Getters_Accessors
/// Getters and accessors for the flocking behavior class

/// Flocking behavior.
/// Agents behaving that way will behave like a flock, or a herd. This basically means that 
/// they will try to stick together as they move, heading in the same direction, and without 
/// bumping into each other.
class dtFlockingBehavior
{
public:
	dtFlockingBehavior();
	dtFlockingBehavior(float desiredSeparation, 
					   float separationWeight, float cohesionWeight, float alignmentWeight, 
					   dtCrowdAgent* agents);
	~dtFlockingBehavior();

	/// Updates the velocity of the given agent.
	///
	/// @param[in]	ag	The agent we want to update.
	/// @param[in]	dt	The time, in seconds, to update the simulation. [Limit: > 0]
	void update(dtCrowdAgent* ag, float dt);
	
	void setAgents(dtCrowdAgent* agents) { m_agents = agents; }

	float m_separationDistance;	///< If the distance between two agents is less than this value, we try to separate them.
	float m_separationWeight;	///< The strength of the separation velocity.
	float m_cohesionWeight;		///< The strength of the cohesion velocity.
	float m_alignmentWeight;	///< The strength of the alignment velocity.

private:	
	/// Prepares the previously computed velocity according the the given agent's parameters (acceleration, max speed, etc.).
	/// Called after the update() method.
	///
	/// @param[in]	velocity	The output velocity.
	/// @param[in]	ag			The agent whose velocity we want to compute.
	/// @param[in]	dt			The time, in seconds, to update the simulation. [Limit: > 0]
	void prepareVelocity(float* velocity, dtCrowdAgent* ag, float dt);

	dtCrowdAgent* m_agents;						///< The list of agents the indices are refering to.

	dtSeparationBehavior* m_separationBehavior;	///< The separation behavior
	dtCohesionBehavior* m_cohesionBehavior;		///< The cohesion behavior
	dtAlignmentBehavior* m_alignmentBehavior;	///< The alignment behavior
};

#endif // DETOURFLOCKINGBEHAVIOR_H