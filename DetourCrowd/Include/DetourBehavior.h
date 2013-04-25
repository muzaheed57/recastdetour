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

#ifndef DETOURBEHAVIORINTERFACE_H
#define DETOURBEHAVIORINTERFACE_H

struct dtCrowdAgent;


/// Interface defining a behavior.
class dtBehavior
{
public:
	dtBehavior();
	~dtBehavior();

	/// Updates the desired velocity of an agent and stores the updates data into the second agent
	///
	/// @param[in]	oldAgent	The agent we want to update.
	/// @param[out]	newAgent	The agent storing the updated version of the oldAgent.
	/// @param[in]	dt			The time, in seconds, to update the simulation. [Limit: > 0]
	virtual void update(dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt) = 0;
};

#endif