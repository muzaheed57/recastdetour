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

#include "DetourSteeringBehavior.h"

struct dtCrowdAgent;
struct dtCrowdAgentEnvironment;
class dtCrowd;


/// Parameters for the separation behavior
/// @ingroup behavior
struct dtSeparationBehaviorParams
{
	unsigned* targetsID;	///< The others agents we want to keep our distances from.
	unsigned nbTargets;		///< The number of targets.
	float distance;			///< From distance from which the agent considers the targets that must be avoided.
	float weight;			///< A coefficient defining how agressively the agent should avoid the targets.
};

/// Implementation of the separation behavior.
///
/// An agent using this behavior will try to keep its distance from one or more targets.
/// The minimum distance to keep from the targets can be specified.
/// @ingroup behavior
class dtSeparationBehavior : public dtSteeringBehavior<dtSeparationBehaviorParams>
{
public:
	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	dtSeparationBehavior(unsigned nbMaxAgents);
	~dtSeparationBehavior();

	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	///
	/// @return		A pointer on a newly allocated behavior
	static dtSeparationBehavior* allocate(unsigned nbMaxAgents);

	/// Frees the given behavior
	///
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtSeparationBehavior* ptr);

	virtual void computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
							  const dtSeparationBehaviorParams& currentParams, dtSeparationBehaviorParams& newParams);
};

#endif // DETOURSEPARATIONBEHAVIOR_H