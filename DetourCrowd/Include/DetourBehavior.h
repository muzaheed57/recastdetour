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

#ifndef DETOURBEHAVIOR_H
#define DETOURBEHAVIOR_H

struct dtCrowdAgent;
class dtCrowdQuery;

/// Interface defining a behavior.
/// @ingroup behavior
class dtBehavior
{
public:
	dtBehavior();
	virtual ~dtBehavior();

	/// Updates the data of an agent so it can behave in the desired way and stores the updated data into the second agent
	///
	/// @param[in]	query		The crowd query object used to access elements of the crowd (agents, navmesh, etc.)
	/// @param[in]	oldAgent	The agent we want to update.
	/// @param[out]	newAgent	The agent storing the updated version of the oldAgent.
	/// @param[in]	dt			The time, in seconds, to update the simulation. [Limit: > 0, otherwise strange things can happen (undefined behavior)]
	virtual void update(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt) = 0;
};

#endif

///////////////////////////////////////////////////////////////////////////

/**

@defgroup behavior Behaviors

Description of the behavior system.

The behaviors are a center part of Recast/Detour, they will allow you to move the agents 
in a specific way.

Some predefined behaviors are provided, but of course the user has the possibility to create his own.

# What does a behavior do?

Basically the goal of a behavior is to compute a new velocity for a given agent. This new velocity will be used 
to update the position / orientation of the agent.

It is important to make difference between the following two properties of an agent:

- The desired velocity (`dtCrowdAgent::desiredVelocity`)
- The current velocity (`dtCrowdAgent::velocity`)

When updating the position of an agent, the desired velocity is used to compute the current velocity which is itself used to determine the next position.
In other words, the user should not update the current velocity of an agent himself (although it can be done). 

Just remember that the current velocity is updated according to the desired velocity.

# Different types of behavior

Several interfaces are provided for behavior creation. The user must inherit from one of these.

__Normal behavior:__ #dtBehavior

This is the highest interface in the behavior hierarchy. When implementing this interface, 
the user has to implement the `dtBehavior::update()` method, which defines how the behavior will affect the data of the 
given agent.

__Steering behavior:__ #dtSteeringBehavior

A steering behavior is a special kind of behavior. A steering behavior works by computing a force that should 
be applied to the current velocity of an agent in order to get the desired velocity. 
This is why the user should doesn't need to implement the `dtSteeringBehavior::update()` method, but instead he needs to define 
the `dtSteeringBehavior::computeForce()` method (and maybe the `dtSteeringBehavior::applyForce()` method as well).

A steering behavior can be parametrized (see below).

__Parametrized behavior:__ #dtParametrizedBehavior

Most of the behaviors cannot work on their own, they need parameters. 
And these parameters can vary from agent to agent (for instance with a following behavior, each agent can follow a different target).
In order to allow your behavior to be parametrized, you need to inherit from `dtParametrizedBehavior` (which implement the `dtBehavior` interface) 
and define the `dtParametrizedBehavior::doUpdate()` method.
This will give you access to a set a methods allowing you to store and access your parameters (which can be represented by any data structure) for every agent.

@note The parameters can also be used to store any data whose lifetime is the same as the behavior's.

# Assign a predefined behavior to an agent

Recast/Detour provides you with a set of predefined behaviors:

- `dtAlignmentBehavior`: The agent moves in the same direction as its targets.
- `dtCohesionBehavior`: The agent moves towards the center of gravity of its targets.
- `dtFlockingBehavior`: Agents move like a flock, or a herd. 
This basically means that they will try to stick together as they move, heading in the same direction, and without bumping into each other.
- `dtArriveBehavior`: The agent moves toward the specified position.
- `dtPathFollowing`: Computes and follows a path from the agent to the specified position using the navigation mesh.
This path can be dynamically updated.
- `dtPipelineBehavior`: This behavior acts like a container for other behaviors. The agent will be updated with every behavior contained in the pipeline. 
This is useful for instance if you want to combine a dtPathFollowing and a dtCollisionAvoidance behavior.
- `dtSeekBehavior`: The agent tries to "catch" its target (another agent). Position prediction can be done for a more efficient path.
- `dtSeparationBehavior`: The agent moves away from the center of gravity of its targets.

Most of these behaviors need some parameters (distance, targets, etc.), and those parameters must be related to the agents.
For each behavior there is a data structure containing the necessary parameters. 
This structure is called `dtBehaviorNameParams` (where `BehaviorName` must be replaced by the name of the behavior). 
Once the parameters are defined, you need to associate them with an agent, then the behavior will know what parameters must be used 
according to the agent it is updating. Here is a sample code:

@code
// Here we create a behavior. 1 is an indication to the behavior, 
// it tells that no more than 1 agents should be using this behavior.
// There can be more or less agents using this behavior than the number you gave, it's not critical.
// This number is just an indication for the allocation size for the HashTable handling the parameters.
dtArriveBehavior arrive(1);

// You also need to associate this behavior to the agent
crowd.setAgentBehavior(&arrive, myAgent.id);

// Now we create the parameters for this behavior. For the Arrive behavior, 
// the already existing structure is dtArriveBehaviorParams.
dtArriveBehaviorParams* arriveParams = arrive.getBehaviorParams(myAgent.id);
float dest[] = {12, 0, 12};

// Now, we need to set the parameters.
arriveParams->distance = 1.f; // Minimal distance to keep between the agent and its target
arriveParams->target = dest; // The target to reach
@endcode

# Create your own behaviors

At some point you might want to create your own behaviors. 
This can easily be done by using the provided interfaces listed above.

Let's take an example. Say we want to create a behavior where an agent would always take the opposite velocity of its target.

First of all, since we are going to need some parameters (for the target), let's create them:

@code
struct MyParams
{
	const int targetID;
};
@endcode

Now we can create our behavior. Since it uses parameters, we need to inherit from `dtParametrizedBehavior`:

@code
class MyBehavior : public dtParametrizedBehavior<MyParams>
{
public:
	explicit MyBehavior(unsigned nbMaxAgent) 
	: dtParametrizedBehavior<MyParams>(nbMaxAgents) 
	{}

	virtual ~MyBehavior() {}

protected:
	virtual void doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
						  const MyParams& currentParams, MyParams& newParams, float dt)
	{
		float* targetVelocity[3];
		dtVcopy(targetVelocity, query.getAgent(currentParams.targetID)->vel);
		dtVscale(targetVelocity, targetVelocity, -1.f);

		dtVset(newAgent.desiredVelocity, targetVelocity);
	}
};
@endcode

Now we have a (simple) behavior ready to be used.

@code
// We create a crowd
dtCrowd crowd;

int nbMaxAgents = 1000;
float maxRadius = 10.f;
crowd.init(nbMaxAgents, maxRadius, navigationMesh);

// Assign agents to the crowd...

// We create the behavior and attach the target to it
MyBehavior b(1);
MyParams* params = b.getBehaviorParams(idAgent);

// We configure the parameters
params->target = crowd.getAgent(idTarget);

// We set the behavior for the agent
crowd.setAgentBehavior(idAgent, &b);
@endcode

Now our behavior has been assigned to the agent. The next time the crowd updates its agents, 
the agent will go in the opposite direction of its target.

*/
