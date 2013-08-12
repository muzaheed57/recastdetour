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

#include "DetourPipelineBehavior.h"

#include "DetourAlloc.h"
#include "DetourCrowd.h"

#include <new>
#include <string.h>

dtPipelineBehavior::dtPipelineBehavior()
	: dtBehavior()
	, m_behaviors(0)
	, m_nbBehaviors(0)
{
}

dtPipelineBehavior::~dtPipelineBehavior()
{
	if (m_behaviors)
	{
		dtFree(m_behaviors);
		m_behaviors = 0;
	}
}

dtPipelineBehavior* dtPipelineBehavior::allocate()
{
	void* mem = dtAlloc(sizeof(dtPipelineBehavior), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtPipelineBehavior();

	return 0;
}

void dtPipelineBehavior::free(dtPipelineBehavior* ptr)
{
	if (!ptr)
		return;

	ptr->~dtPipelineBehavior();
	dtFree(ptr);
	ptr = 0;
}

void dtPipelineBehavior::update(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt)
{
	if (m_behaviors == 0 || m_nbBehaviors == 0)
		return;

	recursiveUpdate(query, oldAgent, newAgent, dt, m_nbBehaviors);
}

void dtPipelineBehavior::recursiveUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt, unsigned remainingBehaviors)
{
	if (remainingBehaviors == 0)
		return;

	dtBehavior* behavior = m_behaviors[m_nbBehaviors - remainingBehaviors];

	if (behavior)
		behavior->update(query, oldAgent, newAgent, dt);

	dtCrowdAgent ag = newAgent;

	recursiveUpdate(query, newAgent, ag, dt, remainingBehaviors - 1);
	newAgent = ag;
}

bool dtPipelineBehavior::setBehaviors(dtBehavior const * const * behaviors, unsigned nbBehaviors)
{
	if (m_behaviors || !behaviors || !nbBehaviors)
	{
		dtFree(m_behaviors);
		m_behaviors = 0;
	}

	// The behavior list has been reseted
	if (!behaviors || !nbBehaviors)
		return true;

	m_behaviors = (dtBehavior**) dtAlloc(sizeof(dtBehavior*) * nbBehaviors, DT_ALLOC_PERM);

	if (!m_behaviors)
		return false;

	memcpy(m_behaviors, behaviors, sizeof(dtBehavior*) * nbBehaviors);

	m_nbBehaviors = nbBehaviors;

	return true;
}
