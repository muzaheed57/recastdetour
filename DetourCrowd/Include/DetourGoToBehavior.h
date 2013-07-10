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

#ifndef DETOURGOTOBEHAVIOR_H
#define DETOURGOTOBEHAVIOR_H

struct dtCrowdAgent;


/// Defines the GoTo behavior.
///
/// This behavior allows the user to set a target, and the agent will try to reach it.
class dtGoToBehavior
{
public:
	dtGoToBehavior();
	~dtGoToBehavior();

	/// Computes the new velocity of the agent
	///
	/// @param[in]	ag		The agent whose velocity must be updated.
	/// @param[out]	force	The computed force that must be applied to the agent's velocity.
	/// @param[in]	dt		The time, in seconds, since the last frame.
	void update(dtCrowdAgent* ag, float* force, const float dt) const;
	
	float m_distance;	///< Minimal distance to keep between the agent and its target.
	float* m_target;	///< The position we want to reach.
};

#endif