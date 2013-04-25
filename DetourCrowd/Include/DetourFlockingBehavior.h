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

#include "DetourSteeringBehavior.h"

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
class dtFlockingBehavior : public dtSteeringBehavior
{
public:
	dtFlockingBehavior();
	dtFlockingBehavior(float separationWeight, float cohesionWeight, float alignmentWeight);
	~dtFlockingBehavior();

	virtual void update(dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt);
	virtual void computeForce(const dtCrowdAgent* ag, float* force);

	float m_separationWeight;		///< The strength of the separation velocity.
	float m_cohesionWeight;			///< The strength of the cohesion velocity.
	float m_alignmentWeight;		///< The strength of the alignment velocity.
	
private:	
	dtSeparationBehavior* m_separationBehavior;	///< The separation behavior
	dtCohesionBehavior* m_cohesionBehavior;		///< The cohesion behavior
	dtAlignmentBehavior* m_alignmentBehavior;	///< The alignment behavior
};

#endif
