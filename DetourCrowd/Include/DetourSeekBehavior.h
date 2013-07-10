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

#ifndef DETOURSEEKBEHAVIOR_H
#define DETOURSEEKBEHAVIOR_H

struct dtCrowdAgent;


/// This class implements the seek behavior.
///
/// An agent with this behavior will try to reach its target.
/// It is possible to set the minimal distance between the agent and the target.
/// (the agent will never get any closer than this distance).
/// Also the agent can have a prediction factor, which means that it will predict the next
/// position of its target (according to its velocity) and head to it.
class dtSeekBehavior
{
public:
	dtSeekBehavior();
	~dtSeekBehavior();

	/// Computes the new velocity of the agent
	///
	/// @param[in]	ag		The agent whose velocity must be updated.
	/// @param[out]	force	The computed force that must be applied to the agent's velocity.
	/// @param[in]	dt		The time, in seconds, since the last frame.
	void update(dtCrowdAgent* ag, float* force, const float dt) const;

	void setTarget(dtCrowdAgent* newTarget) { m_target = newTarget; }
	const dtCrowdAgent* getTarget() const { return m_target; }
	
	float m_distance;				///< Minimal distance to keep between the agent and its target.
	float m_predictionFactor;		///< Used by the agent to predict the next position of the target. The higher the value, The better the prediction. 
									/// Nonetheless a big value is not realistic when agents are close to each other.

private:
	const dtCrowdAgent* m_target;	///< The agent we seek.
};

#endif // DETOURSEEKBEHAVIOR_H
