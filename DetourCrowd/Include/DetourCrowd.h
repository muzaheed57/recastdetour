//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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

#ifndef DETOURCROWD_H
#define DETOURCROWD_H

#include "DetourBehavior.h"
#include "DetourLocalBoundary.h"
#include "DetourNavMeshQuery.h"
#include "DetourPipelineBehavior.h"

class dtObstacleAvoidanceDebugData;
class dtPathFollowing;


/// The maximum number of neighbors that a crowd agent can take into account
/// for steering decisions.
/// @ingroup crowd
static const int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;


/// Provides neighbor data for agents managed by the crowd.
/// @ingroup crowd
/// @see dtCrowdAgent::neis, dtCrowd
struct dtCrowdNeighbour
{
	unsigned idx;	///< The index of the neighbor in the crowd.
	float dist;		///< The distance between the current agent and the neighbor.
};

/// The type of navigation mesh polygon the agent is currently traversing.
/// @ingroup crowd
enum CrowdAgentState
{
	DT_CROWDAGENT_STATE_INVALID,		///< The agent is not in a valid state.
	DT_CROWDAGENT_STATE_WALKING,		///< The agent is traversing a normal navigation mesh polygon.
	DT_CROWDAGENT_STATE_OFFMESH,		///< The agent is traversing an off-mesh connection.
};

/// The environment of an agent
///
/// The environment of an agent contains information about 
/// its neighborhood and the close obstacles.
///
/// @ingroup crowd
struct dtCrowdAgentEnvironment
{
	/// Constructs an environment for a certain number of agents
	explicit dtCrowdAgentEnvironment();
	~dtCrowdAgentEnvironment();

	dtLocalBoundary boundary;									///< The local boundary data for the agent.
	dtCrowdNeighbour neighbors[DT_CROWDAGENT_MAX_NEIGHBOURS];	///< The known neighbors of the agent.
	unsigned nbNeighbors;										///< The number of neighbors.
};

/// Represents an agent managed by a #dtCrowd object.
/// @ingroup crowd
struct dtCrowdAgent
{	
	unsigned id;				///< Unique identifier of the agent
	unsigned char active;		///< 1 if the agent is active, or 0 if the agent is in an unused slot in the agent pool.
	unsigned char state;		///< The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	
	float position[3];			///< The current agent position. [(x, y, z)]
	float desiredVelocity[3];	///< The desired velocity of the agent. [(x, y, z)]
	float velocity[3];			///< The actual velocity of the agent. [(x, y, z)]
			
	dtBehavior* behavior;		///< The behavior used by the agent

	float radius;				///< Agent radius. [Limit: >= 0]
	float height;				///< Agent height. [Limit: > 0]
	float maxAcceleration;		///< Maximum allowed acceleration. [Limit: >= 0]
	float maxSpeed;				///< Maximum allowed speed. [Limit: >= 0]
	float perceptionDistance;	///< 2D distance defining how close a collision element must be before it is considered as an obstacle (Must be greater than 0)

	unsigned char updateFlags;	///< Flags that impact steering behavior. (See: #UpdateFlags)

	float offmeshInitPos[3];
	float offmeshStartPos[3];
	float offmeshEndPos[3];			///< initial, starting and ending position of the animation
	float offmeshElaspedTime;		///< Elapsed time of the animation
	float offmeshTotalTime;			///< Maximum duration of the animation

	void* userData;				///< User defined data attached to the agent.
};

/// Crowd agent update flags.
/// @ingroup crowd
/// @see dtCrowdAgent::updateFlags
enum UpdateFlags
{
	DT_CROWD_ANTICIPATE_TURNS = 1,
	DT_CROWD_OBSTACLE_AVOIDANCE = 2,
	DT_CROWD_SEPARATION = 4,
	DT_CROWD_OPTIMIZE_VIS = 8,			///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	DT_CROWD_OPTIMIZE_TOPO = 16,		///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
};

/// Utility class used to get access to some useful elements of the crowd
/// @ingroup crowd
class dtCrowdQuery
{
public:
	/// Constructor
	///
	/// @param[in]	maxAgents	The maximum number of agents for the crowd.
	/// @param[in]	agents		The agents of the crowd.
	/// @param[in]	env			The environments of the agents of the crowd.
	dtCrowdQuery(unsigned maxAgents, const dtCrowdAgent* agents, const dtCrowdAgentEnvironment* env);

	~dtCrowdQuery();

	/// @name Data access
	/// @{
	float* getQueryExtents();
	dtNavMeshQuery* getNavMeshQuery();
	dtQueryFilter* getQueryFilter();

	const float* getQueryExtents() const;
	const dtNavMeshQuery* getNavMeshQuery() const;
	const dtQueryFilter* getQueryFilter() const;

	/// Gets a const access to the specified agent from the pool.
	///	 @param[in]		id	The agent id.
	/// @return The requested agent.
	const dtCrowdAgent* getAgent(const unsigned id) const;

	/// Gets the specified agent from the pool.
	/// The required agent's data are copied into the given dtCrowdAgent object
	///	 @param[in]		ag	The dtCrowdAgent object into which the data will be copied.
	///	 @param[in]		id	The agent id.
	void fetchAgent(dtCrowdAgent& ag, unsigned id) const;

	/// Gets the agents matching the given ids
	/// @param[in]	ids			The list of ids
	/// @param[in]	size		The size of the list of indices (must match the size of the agents list)
	/// @param[out]	agents		The list of agents where the results will be stored
	/// @return Returns the number of agents found and put into the array
	unsigned getAgents(const unsigned* ids, unsigned size, const dtCrowdAgent** agents) const;

	/// Gets the environment of the given agent
	/// @param[in]	id	The id of the agent
	/// @return Returns the environment of the given agent
	const dtCrowdAgentEnvironment* getAgentEnvironment(unsigned id) const;

	/// Get the offMesh connection the agent is on or close to.
	/// The user can specify an additional distance if he wants to know if an offMesh connection
	/// is located at a certain distance of the agent.
	/// @param[in]	id		The id of the agent
	/// @param[in]	dist	The additional distance
	/// @return 0 if no offMesh connection have been detected. Otherwise returns the offMesh connection
	dtOffMeshConnection* getOffMeshConnection(unsigned id, float dist = 0.f) const;
	/// @}

private:
	float m_ext[3];								///< The query filters used for navigation queries.
	dtNavMeshQuery* m_navMeshQuery;				///< Used to perform queries on the navigation mesh.
	dtQueryFilter m_filter;						///< Defines polygon filtering and traversal costs for navigation mesh query operations.
	const dtCrowdAgent* m_agents;				///< The agents of the crowd
	unsigned m_maxAgents;						///< Max number of agents in the crowd
	const dtCrowdAgentEnvironment* m_agentsEnv;	///< The environments of the agents
};

/// Class containing and handling the agents of the simulation.
///
/// Every modification to any agent cannot be done directly, it has to be done using the provided methods.
///
/// @ingroup crowd
class dtCrowd
{
	dtCrowdQuery* m_crowdQuery;				///< CrowdQuery object for accessing data
	dtCrowdAgentEnvironment* m_agentsEnv;	///< The environments of the agents

	unsigned m_maxAgents;					///< The maximum number of agents contained by the crowd
	unsigned m_nbActiveAgents;				///< The number of active agents
	dtCrowdAgent* m_agents;					///< The agents of the crowd
	dtCrowdAgent** m_activeAgents;			///< the actives agents of the crowd
	unsigned* m_agentsToUpdate;				///< indexes of all agents
		
	float m_maxAgentRadius;					///< Maximal radius for an agent
	unsigned m_maxCommonNodes;				///< Maximal number of search nodes for the navigation mesh

	float** m_disp;							///< Used to prevent agents from bumping into each other
	
	/// Returns the index of the given agent
	inline unsigned getAgentIndex(const dtCrowdAgent* agent) const  { return agent - m_agents; }

	/// Fetch the agent of the given id if he is active.
	/// param[out]	ag	The agent corresponding to the given id
	/// param[in]	id	the id of the agent we want to fetch
	/// @return	False if the agent is not active or if the id is out of bound. True otherwise
	bool getActiveAgent(dtCrowdAgent** ag, int id);

	/// Gets the neighbors of  the given agent.
	/// Uses the field of view of the agent for that.
	/// The neighbors will be stored in the agent environment
	///
	/// @param[in]	id	ID of the agent
	/// @return	The number of neighbors found
	unsigned computeNeighbors(unsigned id);

	/// Cleans the crowd so it can be used for a fresh start
	void purge();
	
public:
	dtCrowd();
	~dtCrowd();
	
	/// Initializes the crowd.  
	///  @param[in]		maxAgents		The maximum number of agents the crowd can manage. [Limit: >= 1]
	///  @param[in]		maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
	///  @param[in]		nav				The navigation mesh to use for planning.
	/// @return True if the initialization succeeded.
	bool init(const unsigned maxAgents, const float maxAgentRadius, dtNavMesh* nav);	

	/// @name Data access
	/// @{

	/// Gets a const access to the specified agent from the pool.
	///	 @param[in]		id	The agent id.
	/// @return The requested agent.
	const dtCrowdAgent* getAgent(const unsigned id) const;

	/// Gets the specified agent from the pool.
	/// The required agent's data are copied into the given dtCrowdAgent object
	///	 @param[out]	ag	The dtCrowdAgent object into which the data will be copied.
	///	 @param[in]		id	The agent id.
	void fetchAgent(dtCrowdAgent& ag, unsigned id) const;
	
	/// Gets the agents matching the given ids
	/// @param[in]	ids			The list of ids
	/// @param[in]	size		The size of the list of indices (must match the size of the agents list)
	/// @param[out]	agents		The list of agents where the results will be stored
	/// @return Returns the number of agents found and put into the array
	unsigned getAgents(const unsigned* ids, unsigned size, const dtCrowdAgent** agents) const;

	/// Gets the environment of the given agent
	/// @param[in]	id	The id of the agent
	/// @return Returns the environment of the given agent
	const dtCrowdAgentEnvironment* getAgentEnvironment(unsigned id) const;

	/// Gets the active agents int the agent pool.
	///  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
	///  @param[in]		maxAgents	The size of the crowd agent array.
	/// @return The number of agents returned in @p agents.
	unsigned getActiveAgents(const dtCrowdAgent** agents, const unsigned maxAgents);
	
	/// Gets the maximum number of agents that can be managed by the object.
	const unsigned getAgentCount() const;

	const dtCrowdQuery* getCrowdQuery() const { return m_crowdQuery; }
	dtCrowdQuery* getCrowdQuery() { return m_crowdQuery; }
	/// @}

	/// @name Data modifiers
	/// @{

	/// Copies the data of the given agent into its equivalent in the crowd
	/// In order to know which agent must receive the data, we refer to the id of the given agent
	/// @return False if the id of the agent could not be matched or if there are some inconsistencies 
	/// with the data of the given agent. False otherwise.
	bool applyAgent(const dtCrowdAgent& ag);

	/// Sets the behavior of the agent referenced by the given id
	/// @param[in]	behavior	The behavior to affect
	/// @param[in]	id			the agent of the agent to edit
	bool setAgentBehavior(unsigned id, dtBehavior* behavior);

	/// Adds a new agent to the crowd.
	/// The data of the given agent will be copied into the crowd's pool.
	/// The position of the agent in the pool will be determined according to its id.
	///  @param[out]	agent	The agent whose data should be copied.
	///  @param[in]		pos		The requested position of the agent. [(x, y, z)]
	/// @return False if the id of the agent is incorrect, True otherwise.
	bool addAgent(dtCrowdAgent& agent, const float* pos);

	/// Removes the agent of the given id from the crowd.
	///  @param[in]		id		The agent id.
	void removeAgent(unsigned id);

	/// Moves the agent to the given position if possible.
	/// 
	/// @param[in]	id			The id of the agent.
	/// @param[in]	position	The new desired position.
	/// @return		False if the position is outside the navigation mesh or if the index is out of bound. True otherwise.
	bool updateAgentPosition(unsigned id, const float* position);
	/// @}

	/// Indicates whether the agent is moving or not.
	/// An agent is moving when:
	/// - its desired speed is > 0
	/// - its velocity is not a nil vector
	bool agentIsMoving(const dtCrowdAgent& ag) const;
	
	/// @name Updating the crowd
	/// @{

	/// Updates the steering and positions of the agents whose indices may be given by the user.
	/// If no indices are given, then the method updates every agent.
	///  @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[in]		agentsIdx	The list of the indices of the agents we want to update. [Opt]
	///  @param[in]		nbIdx		Size of the list of indices. [Opt]
	void update(const float dt, unsigned* agentsIdx = 0, unsigned nbIdx = 0);

	/// Updates the velocity of the agents whose indices may be given by the user (but not their position).
	/// If no indices are given, then the method updates every agent.
	///  @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[in]		agentsIdx	The list of the indices of the agents we want to update. [Opt]
	///  @param[in]		nbIdx		Size of the list of indices. [Opt]
	void updateVelocity(const float dt, unsigned* agentsIdx = 0, unsigned nbIdx = 0);

	/// Updates the positions of the agents whose indices may be given by the user (but not their velocity).
	/// If no indices are given, then the method updates every agent.
	///  @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[in]		agentsIdx	The list of the indices of the agents we want to update. [Opt]
	///  @param[in]		nbIdx		Size of the list of indices. [Opt]
	void updatePosition(const float dt, unsigned* agentsIdx = 0, unsigned nbIdx = 0);

	/// Updates the environment of the given agents. 
    /// Updates the proximity grid and registers every agent's neighbor.
	/// If no indices are given, then the method updates every agent.
	///  @param[in]		agentsIdx	The list of the indices of the agents we want to update. [Opt]
	///  @param[in]		nbIdx		Size of the list of indices. [Opt]
	void updateEnvironment(unsigned* agentsIdx = 0, unsigned nbIdx = 0);
	/// @}
};

/// Allocates a crowd object using the Detour allocator.
/// @return A crowd object that is ready for initialization, or null on failure.
///  @ingroup crowd
dtCrowd* dtAllocCrowd();

/// Frees the specified crowd object using the Detour allocator.
///  @param[in]		ptr		A crowd object allocated using #dtAllocCrowd
///  @ingroup crowd
void dtFreeCrowd(dtCrowd* ptr);


#endif // DETOURCROWD_H

///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@defgroup crowd Crowd Management

This module is about handling multiple agents.

`dtCrowd` is the class representing a crowd. 

A crowd handles every agent it contains. 
You can think of it as a big container for agents.

You can see `dtCrowd` as an interface between you and the agents it contains.
You cannot modify the agents inside the crowd directly, 
the interface will only grant you a constant access to the data.

Through `dtCrowd` you can do several things:

- Have constant access to the agents
- Have access to the data the crowd uses to update the agents (navigation mesh, proximity grid, environment, etc.)
- Add / Remove / Edit agents inside the crowd via some specific methods
- Update the crowd. This will update every agents using their behaviors. When updating an agent you can 
update its position, velocity, environment or all of those.


# How to create a crowd

Here is a sample code for creating a crowd:

@code
// Creation of the crowd.
dtCrowd crowd;
// How many agents can the crowd contain?
int nbMaxAgents = 1000;
// The maximum radius for an agent
float maxRadius = 10.f;
// How close should an element be before being considered as an obstacle?
float collisionRange = 4.f;

// Note: the navigation mesh must already be initialized
// The navigation mesh will be used by the agents for navigation
crowd.init(nbMaxAgents, mawRadius, navigationMesh, collisionRange);
@endcode

You now have a crowd ready to work, but empty...

# Agents

`dtCrowdAgent` is the class representing an agent.

An agent is an entity inside a crowd with several properties. The most important of these is the `dtCrowdAgent::id` property.

@warning Each agent is identified by its id, and the crowd handles its agents using these ids. Therefore the user __should 
not__ modify the id of an agent by himself. The ids are generated by crowd and are unique.

# Add an agent to the crowd

When you create a crowd, you create the agents at the same time. Indeed the crowd will create N agents, 
N being the maximum of agents you specified when creating the crowd, and set them as inactive. 
If inactive, an agent won't be taken into account during the process of updating the crowd.

Here is how you can toggle an agent as active/inactive:

@code
// The position you want to place your agent at
float position[] = {0, 0, 0};
dtCrowdAgent agent;

if (crowd.addAgent(agent, position) == false)
	// Error The agent could not be added to the crowd

// At this point, agent has been added and set as active. Its data (and id) have been updated.

// You can also remove the agent from the crowd, which will set him as inactive
crowd.removeAgent(ag.id);
@endcode

With the method `dtCrowd::addAgent()` we can toggle an agent as active an place it at a specific point on the navigation mesh. 
The method will copy the data of the newly created agent into the agent structure you provided. 
You can use the id of the agent later on to get access to it thanks to the method `dtCrowd::getAgent()`.

Your crowd now contains one agent.

# Access and edit existing agents

`dtCrowd` does not allow you to directly edit the agents. You need to get a copy of the agent, edit it, then send the modifications to the crowd.
In any case, you will need the id of an agent in order to retrieve it.

Here is a code illustrating the different ways to access agents:

@code
// You can get a const access to an agent with its id
const dtCrowdAgent* ag = crowd.getAgent(id);

// You can fetch an agent and copy the data into another agent
dtCrowdAgent myAgent;
crowd.fetchAgent(myAgent, id);

// You can also get several agents at the same time
int ids[] = {1, 2, 3, 6};
const dtCrowdAgent* agents[4];
int nbAgentsFound = crowd.getAgents(ids, 4, agents);
@endcode

Now that you have accessed your agent, you might want to modify some of its properties, and then send the changes to the crowd, 
here is how this can be acheived:

@code
// Fetch the agent you want to edit
dtCrowdAgent ag;
crowd.fetchAgent(ag, id); // The data have been copied into ag (including the id)

// Edit the properties
ag.maxSpeed *= 2;
ag.radius = 10;
ag.height = 500;
// ...

// You can now send the changes to the crowd
crowd.applyAgent(ag);
@endcode

@warning `dtCrowd` uses the id when looking for and editing an agent, therefore be careful not the change the id property of an agent, 
otherwise the crowd might not update the right agent.

# Update the crowd

When you update the crowd, by default, all the active agents inside this crowd will be updated. 
But maybe you just want some of them to be updated. This can be done by providing a list of agents' ids.

@note When updating the agents, the crowd will use the behaviors assigned to them. See the @ref behavior module for more details.

@code
float position1[] = {0, 0, 0};
float position2[] = {1, 0, 1};
float position3[] = {2, 0, 2};

dtCrowdAgent ag1, ag2, ag3;

if (!crowd.addAgent(ag1, position1) || !crowd.addAgent(ag2, position2) || !crowd.addAgent(ag3, position3))
	return false;

// ...

// Updates every active agents inside the crowd.
crowd.update(dt); // dt is the delta time to update the simulation.

int list[2];
list[0] = ag2.id;
list[1] = ag3.id;

// Just the agents 2 and 3 will be updated.
crowd.update(dt, list, 2);

// Just the agent corresponding to index2 will be updated (since we say the list has a size of 1).
crowd.update(dt, list, 1);
@endcode

Crowds updating is a three parts process, for each agent we do the following:

- __Updating the environment:__ Updates the proximity grid and registers every agent's neighbor. This will take into account 
things like other agents, boundaries, obstacles, etc. The range of the environment depends on the agent's property `dtCrowd::collisionQueryRange`.
- __Updating the velocity:__ Updates the velocity of the agent. This uses the behaviors assigned to the agent. These behaviors will compute a new velocity 
for the agent.
- __Updating the position:__ Updates the position of the agent using the previously computed velocity.

The calls order should be this:

- Update the environment so the agent knows what is around him,
- Update the velocity using a behavior,
- Update the position using the new velocity.

You can individually call these updates using the following methods:

- `dtCrowd::updateEnvironment()`
- `dtCrowd::updateVelocity()`
- `dtCrowd::updatePosition()`

@note Calling the method `dtCrowd::update()` will call all three methods listed above.

# Other features

## Change the position of an agent

Of course the position of an agent will be updated trough its behavior(s), but sometimes you might want to 
set your agent at a given position instantly, without the constraints of a behavior (for instance your agent 
have walked on a teleporter). This can be done using the `dtCrowd::updateAgentPosition()` method:

@code
float newPosition[] = {10, 0, 0};

if (crowd.updateAgentPosition(agentId, newPosition))
	// Done, your agent has moved
else
	// Something went wrong, either the position is invalid or the given id was out of bounds
@endcode

## Access the environment of an agent

The environment of an agent contains many informations that can be useful for some behavior 
(for instance it is used by the behavior `dtCollisionAvoidance`).

You can retreive the environment of an agent by using the method `dtCrowd::getAgentEnvironment`.

## The Crowd Query

`dtCrowdQuery` contains informations that `dtCrowd` uses and that might be useful for the user.
It contains things like the proximity grid, the animations of the agents, the `dtNavMeshQuery`, etc.

Since it might often be useful, `dtCrowd` provides an easy access via the method `dtCrowd::getCrowdQuery()`.

*/
