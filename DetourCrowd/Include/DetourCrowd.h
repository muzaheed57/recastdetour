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
#include "DetourProximityGrid.h"


class dtSeekBehavior;

class dtFlockingBehavior;
class dtGoToBehavior;
class dtSeparationBehavior;
class dtAlignmentBehavior;
class dtCohesionBehavior;
class dtObstacleAvoidanceDebugData;


/// The maximum number of neighbors that a crowd agent can take into account
/// for steering decisions.
/// @ingroup crowd
static const int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;


/// Provides neighbor data for agents managed by the crowd.
/// @ingroup crowd
/// @see dtCrowdAgent::neis, dtCrowd
struct dtCrowdNeighbour
{
	int idx;		///< The index of the neighbor in the crowd.
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

struct dtCrowdAgentAnimation
{
	unsigned char active;
	float initPos[3], startPos[3], endPos[3];
	dtPolyRef polyRef;
	float t, tmax;
};

enum MoveRequestState
{
	DT_CROWDAGENT_TARGET_NONE = 0,
	DT_CROWDAGENT_TARGET_FAILED,
	DT_CROWDAGENT_TARGET_VALID,
	DT_CROWDAGENT_TARGET_REQUESTING,
	DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
	DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
	DT_CROWDAGENT_TARGET_VELOCITY,
};

struct dtCrowdAgentEnvironment
{
	dtCrowdAgentEnvironment(int nbMaxAgents);
	~dtCrowdAgentEnvironment();

	/// The local boundary data for the agent.
	dtLocalBoundary boundary;

	/// The known neighbors of the agent.
	dtCrowdNeighbour neighbors[DT_CROWDAGENT_MAX_NEIGHBOURS];

	/// The number of neighbors.
	int nbNeighbors;
};

/// Represents an agent managed by a #dtCrowd object.
/// @ingroup crowd
struct dtCrowdAgent
{
	/// Unique identifier of the agent
	int id;

	/// 1 if the agent is active, or 0 if the agent is in an unused slot in the agent pool.
	unsigned char active;

	/// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	unsigned char state;

	/// The environment of the agent
	//dtCrowdAgentEnvironment environment;
	
	/// The desired speed.
	float desiredSpeed;

	float npos[3];		///< The current agent position. [(x, y, z)]
	float disp[3];
	float dvel[3];		///< The desired velocity of the agent. [(x, y, z)]
	float vel[3];		///< The actual velocity of the agent. [(x, y, z)]
			
	dtBehavior* behavior;

	float radius;					///< Agent radius. [Limit: >= 0]
	float height;					///< Agent height. [Limit: > 0]
	float maxAcceleration;			///< Maximum allowed acceleration. [Limit: >= 0]
	float maxSpeed;					///< Maximum allowed speed. [Limit: >= 0]

	/// Flags that impact steering behavior. (See: #UpdateFlags)
	unsigned char updateFlags;

	/// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	float collisionQueryRange;

	/// User defined data attached to the agent.
	void* userData;
};

/// Crowd agent update flags.
/// @ingroup crowd
/// @see dtCrowdAgentParams::updateFlags
enum UpdateFlags
{
	DT_CROWD_ANTICIPATE_TURNS = 1,
	DT_CROWD_OBSTACLE_AVOIDANCE = 2,
	DT_CROWD_SEPARATION = 4,
	DT_CROWD_OPTIMIZE_VIS = 8,			///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	DT_CROWD_OPTIMIZE_TOPO = 16,		///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
};

struct dtCrowdAgentDebugInfo
{
	int idx;
	float optStart[3], optEnd[3];
	dtObstacleAvoidanceDebugData* vod;
};

class dtPathFollowing;
class dtCollisionAvoidance;

class dtCrowdQuery
{
public:
	dtCrowdQuery(int maxAgents);

	~dtCrowdQuery();

	dtCrowdAgentAnimation* getAgentsAnims();
	float* getQueryExtents();
	dtNavMeshQuery* getNavMeshQuery();
	dtQueryFilter* getQueryFilter();
	dtProximityGrid* getProximityGrid();

private:
	dtCrowdAgentAnimation* m_agentAnims;
	float m_ext[3];
	dtNavMeshQuery* m_navMeshQuery;			///< Used to perform queries on the navigation mesh.
	dtQueryFilter m_filter;					///< Defines polygon filtering and traversal costs for navigation mesh query operations.
	dtProximityGrid* m_grid;
	int m_maxAgents;
};

/// Provides local steering behaviors for a group of agents. 
/// @ingroup crowd
class dtCrowd
{
	dtCrowdQuery* m_crowdQuery;
	dtCrowdAgentEnvironment* m_agentsEnv;

	int m_maxAgents;
	int m_nbActiveAgents;
	dtCrowdAgent* m_agents;
	dtCrowdAgent** m_activeAgents;
	int* m_agentsToUpdate;
		
	int m_maxPathResult;
	float m_maxAgentRadius;
	int m_maxCommonNodes;
	int m_maxPathQueueNodes;

	inline int getAgentIndex(const dtCrowdAgent* agent) const  { return agent - m_agents; }
	bool getActiveAgent(dtCrowdAgent** ag, int id);

	void purge();
	
public:
	dtCrowd();
	~dtCrowd();
	
	/// Initializes the crowd.  
	///  @param[in]		maxAgents		The maximum number of agents the crowd can manage. [Limit: >= 1]
	///  @param[in]		maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
	///  @param[in]		nav				The navigation mesh to use for planning.
	/// @return True if the initialization succeeded.
	bool init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav);
	
	/// Gets the specified agent from the pool.
	///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return The requested agent.
	const dtCrowdAgent* getAgent(const int idx) const;

	/// Gets the specified agent from the pool.
	///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return The requested agent.
	dtCrowdAgent* getAgent(const int idx);

	const dtCrowdAgentEnvironment* getAgentsEnvironment() const;

	/// The maximum number of agents that can be managed by the object.
	/// @return The maximum number of agents.
	const int getAgentCount() const;

	/// Gets the list of agents inside the crowd (active or not)
	dtCrowdAgent* getAgents() const { return m_agents; }

	/// Indicates whether the agent is moving or not.
	/// An agent is moving when:
	/// - its desired speed is > 0
	/// - its velocity is not a nil vector
	bool agentIsMoving(int index) const;

	/// Adds a new agent to the crowd.
	///  @param[in]		pos		The requested position of the agent. [(x, y, z)]
	/// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
	int addAgent(const float* pos);

	/// Removes the agent from the crowd.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	void removeAgent(const int idx);

	/// Gets the active agents into the agent pool.
	///  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
	///  @param[in]		maxAgents	The size of the crowd agent array.
	/// @return The number of agents returned in @p agents.
	int getActiveAgents(dtCrowdAgent** agents, const int maxAgents);
	
	/// Gets the agents into the agent pool.
	///  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
	void getAllAgents(dtCrowdAgent** agents);

	/// Updates the steering and positions of the agents whose indices may be given by the user.
	/// If no indices are given, then the method updates every agent.
	///  @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[in]		agentsIdx	The list of the indices of the agents we want to update. [Opt]
	///  @param[in]		nbIdx		Size of the list of indices. [Opt]
	void update(const float dt, int* agentsIdx = 0, int nbIdx = 0);

	/// Updates the velocity of the agents whose indices may be given by the user (but not their position).
	/// If no indices are given, then the method updates every agent.
	///  @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[in]		agentsIdx	The list of the indices of the agents we want to update. [Opt]
	///  @param[in]		nbIdx		Size of the list of indices. [Opt]
	void updateVelocity(const float dt, int* agentsIdx = 0, int nbIdx = 0);

	/// Updates the positions of the agents whose indices may be given by the user (but not their velocity).
	/// If no indices are given, then the method updates every agent.
	///  @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[in]		agentsIdx	The list of the indices of the agents we want to update. [Opt]
	///  @param[in]		nbIdx		Size of the list of indices. [Opt]
	void updatePosition(const float dt, int* agentsIdx = 0, int nbIdx = 0);

	/// Updates the environment of the given agents. 
    /// Updates the proximity grid and registers every agent's neighbor.
	/// If no indices are given, then the method updates every agent.
	///  @param[in]		agentsIdx	The list of the indices of the agents we want to update. [Opt]
	///  @param[in]		nbIdx		Size of the list of indices. [Opt]
	void updateEnvironment(int* agentsIdx = 0, int nbIdx = 0);

	/// Moves the agent to the given position if possible.
	/// 
	/// @param[in]	index		The index of the agent (must be >= 0 and < maxAgents)
	/// @param[in]	position	The new desired position.
	/// @return		False if the position is outside the navigation mesh or if the index is out of bound. True otherwise.
	bool updateAgentPosition(int index, const float* position);

	const dtCrowdQuery* getCrowdQuery() const { return m_crowdQuery; }
	dtCrowdQuery* getCrowdQuery() { return m_crowdQuery; }
	
	int getNbMaxAgents() { return m_maxAgents; }
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

@defgroup crowd Crowd

Members in this module implement local steering and dynamic avoidance features.

The crowd is the big beast of the navigation features. It not only handles a 
lot of the path management for you, but also local steering and dynamic 
avoidance between members of the crowd. I.e. It can keep your agents from 
running into each other.

Main class: #dtCrowd

The #dtNavMeshQuery and #dtPathCorridor classes provide perfectly good, easy 
to use path planning features. But in the end they only give you points that 
your navigation client should be moving toward. When it comes to deciding things 
like agent velocity and steering to avoid other agents, that is up to you to 
implement. Unless, of course, you decide to use #dtCrowd.

Basically, you add an agent to the crowd, providing various configuration 
settings such as maximum speed and acceleration. You also provide a local 
target to more toward. The crowd manager then provides, with every update, the 
new agent position and velocity for the frame. The movement will be 
constrained to the navigation mesh, and steering will be applied to ensure 
agents managed by the crowd do not collide with each other.

This is very powerful feature set. But it comes with limitations.

The biggest limitation is that you must give control of the agent's position 
completely over to the crowd manager. You can update things like maximum speed 
and acceleration. But in order for the crowd manager to do its thing, it can't 
allow you to constantly be giving it overrides to position and velocity. So 
you give up direct control of the agent's movement. It belongs to the crowd.

The second biggest limitation revolves around the fact that the crowd manager 
deals with local planning. So the agent's target should never be more than 
256 polygons aways from its current position. If it is, you risk 
your agent failing to reach its target. So you may still need to do long 
distance planning and provide the crowd manager with intermediate targets.

Other significant limitations:

- All agents using the crowd manager will use the same #dtQueryFilter.
- Crowd management is relatively expensive. The maximum agents under crowd 
  management at any one time is between 20 and 30.  A good place to start
  is a maximum of 25 agents for 0.5ms per frame.

@note This is a summary list of members.  Use the index or search 
feature to find minor members.

@struct dtCrowdAgentParams
@see dtCrowdAgent, dtCrowd::addAgent(), dtCrowd::updateAgentParameters()

@var dtCrowdAgentParams::obstacleAvoidanceType
@par

#dtCrowd permits agents to use different avoidance configurations.  This value 
is the index of the #dtObstacleAvoidanceParams within the crowd.

@see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), 
	 dtCrowd::getObstacleAvoidanceParams()

@var dtCrowdAgentParams::collisionQueryRange
@par

Collision elements include other agents and navigation mesh boundaries.

This value is often based on the agent radius and/or maximum speed. E.g. radius * 8

@var dtCrowdAgentParams::pathOptimizationRange
@par

Only applicalbe if #updateFlags includes the #DT_CROWD_OPTIMIZE_VIS flag.

This value is often based on the agent radius. E.g. radius * 30

@see dtPathCorridor::optimizePathVisibility()

@var dtCrowdAgentParams::separationWeight
@par

A higher value will result in agents trying to stay farther away from each other at 
the cost of more difficult steering in tight spaces.

*/
