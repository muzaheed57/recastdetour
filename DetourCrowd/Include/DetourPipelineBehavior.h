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

#ifndef DETOURPIPELINEBEHAVIOR_H
#define DETOURPIPELINEBEHAVIOR_H

#include "DetourBehavior.h"

struct dtCrowdAgent;
class dtCrowdQuery;

/// Behavior having the ability to contain other behaviors.
/// 
/// The behavior will be called one after another, this means that there is a risk that 
/// a behavior might erase the modifications done by the previous one (depending on how they were implemented).
/// Also, the order in which you put your behaviors into the pipeline matters.
/// @ingroup behavior
class dtPipelineBehavior : public dtBehavior
{
public:
	dtPipelineBehavior();
	~dtPipelineBehavior();

	/// Creates an instance of the behavior
	/// @return		A pointer on a newly allocated behavior
	static dtPipelineBehavior* allocate();

	/// Frees the given behavior
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtPipelineBehavior* ptr);

	virtual void update(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt);

	/// Affects the given behaviors to the pipeline
	/// The behaviors are not copied, but their references are.
	/// In order to clear the behaviors of the pipeline, give the 0 value to one of the parameters.
	///
	/// @param[in]	behaviors	The behaviors we want to affect to the pipeline
	/// @param[in]	nbBehaviors	The number of behaviors we want to affect to the pipeline
	///
	/// @return	False if the memory for the behaviors could not be allocated. True otherwise
	bool setBehaviors(dtBehavior const * const * behaviors, unsigned nbBehaviors);

private:
	void recursiveUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt, unsigned remainingBehaviors);

	dtBehavior** m_behaviors;	///< The behaviors affected to the pipeline
	int m_nbBehaviors;			///< The number of behaviors affected to the pipeline
};

#endif