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

#ifndef DETOURPATHFOLLOWING_H
#define DETOURPATHFOLLOWING_H

#include "DetourPathQueue.h"
#include "DetourNavMeshQuery.h"

class dtNavMesh;
struct dtCrowdAgent;
struct dtCrowdAgentDebugInfo;
struct dtCrowdAgentAnimation;


/// Defines a behavior for pathfollowing.
///
/// Using a navigation mesh, the pathfollowing behavior works on 
/// a list of agent in order to update their velocity so they can
/// reach their goal.
class dtPathFollowing
{
public:
	dtPathFollowing();
	~dtPathFollowing();

	/// Allocates an object of type dtPathFollowing using the Detour allocator.
	///
	/// @return 0 if the allocation failed, a pointer to the memory allocated otherwise.
	static dtPathFollowing* allocate();

	/// Destroys an object of type dtPathFollowing using the Detour allocator.
	///
	/// @param	ptr		The pointer the the area we want to release.
	static void destroy(dtPathFollowing* ptr);

	/// Initializes the behavior.
	///
	/// Must be called before using the behavior.
	/// @param[in]		navMesh		The navigation mesh representing the navigation search space for the agents.
	/// @param[in]		ext			The search distance along each axis. [(x, y, z)]
	/// @param[in]		maxPathRes	Max number of path results.
	/// @param[in]		agents		The list of agents (active or not) inside the crowd.
	/// @param[in]		maxAgents	Size of the list of agents.
	///
	/// @return True if the initialization succeeded.
	bool init(dtNavMesh* navMesh, float* ext, int maxPathRes, dtCrowdAgent* agents, int maxAgents);

	/// Cleans the class before destroying
	void purge();

	/// Update the velocity of the given agents.
	/// 
	/// @param[in]		agents			List of active agents.
	/// @param[in]		agentsIdx		The list of the indexes of the agents we want to update.
	/// @param[in]		nbIdx			Number of agents to work on.
	/// @param[in]		m_agentAnims	The animations for each agent.
	/// @param[in]		dt				The time, in seconds, to update the simulation. [Limit: > 0]
	/// @param[in]		debug			A debug object to load with debug information. [Opt]
	void update(dtCrowdAgent** agents, int* agentsIdx, const int nbIdx, dtCrowdAgentAnimation* m_agentAnims, 
				const float dt, dtCrowdAgentDebugInfo* debug = 0);
		
	/// Get the navigation mesh
	const dtNavMeshQuery* getNavMeshQuery() const { return m_navMeshQuery; }
	dtNavMeshQuery* getNavMeshQuery() { return m_navMeshQuery; }

	/// Get the query filter
	const dtQueryFilter* getQueryFilter() const { return &m_filter; }
	dtQueryFilter* getQueryFilter() { return &m_filter; }

	/// Get the search distance along each axis. [(x, y, z)]
	const float* getQueryExtent() const { return m_ext; }
	float* getQueryExtent() { return m_ext; }

	/// Get the path queue
	const dtPathQueue* getPathQueue() const { return &m_pathQueue; }
	dtPathQueue* getPathQueue() { return &m_pathQueue; }

	/// Get the path result
	const dtPolyRef* getPathRes() const { return m_pathResult; }
	dtPolyRef* getPathRes() { return m_pathResult; }

private:
	/// Checks that the given agents still have valid paths.
	/// 
	/// @param[in]		agents			List of active agents.
	/// @param[in]		agentsIdx		The list of the indexes of the agents we want to update.
	/// @param[in]		nagents			Number of agents to work on.
	/// @param[in]		dt				The time, in seconds, to update the simulation. [Limit: > 0]
	void checkPathValidity(dtCrowdAgent** agents, int* agentsIdx, const int nagents, const float dt);

	/// Update async move request and path finder.
	void updateMoveRequest();

	/// Optimize path topology.
	/// 
	/// @param[in]		agents			List of active agents.
	/// @param[in]		agentsIdx		The list of the indexes of the agents we want to update.
	/// @param[in]		nagents			Number of agents to work on.
	/// @param[in]		dt				The time, in seconds, to update the simulation. [Limit: > 0]
	void updateTopologyOptimization(dtCrowdAgent** agents, int* agentsIdx, const int nagents, const float dt);

	/// Performs some checking and optimization on the agents.
	///
	/// @param[in]		agents			List of active agents.
	/// @param[in]		agentsIdx		The list of the indexes of the agents we want to update.
	/// @param[in]		nagents			Number of agents to work on.
	/// @param[in]		dt				The time, in seconds, to update the simulation. [Limit: > 0]
	void prepare(dtCrowdAgent** agents, int* agentsIdx, const int nagents, const float dt);

	/// Computes the new velocity of the given agents according to their environment.
	///
	/// @param[in]		agents			List of active agents.
	/// @param[in]		agentsIdx		The list of the indexes of the agents we want to update.
	/// @param[in]		nagents			Number of agents to work on.
	void getVelocity(dtCrowdAgent** agents, int* agentsIdx, const int nbIdx);

	/// Finds the next corner the given agents should aim to
	/// 
	/// @param[in]		agents			List of active agents.
	/// @param[in]		agentsIdx		The list of the indexes of the agents we want to update.
	/// @param[in]		nbIdx			Number of agents to work on.
	/// @param[in]		debug			A debug object to load with debug information. [Opt]
	void getNextCorner(dtCrowdAgent** agents, int* agentsIdx, const int nbIdx, dtCrowdAgentDebugInfo* debug = 0);

	/// Checks whether the given are on an offmesh connection. If so, the path is adjusted.
	/// 
	/// @param[in]		agents			List of active agents.
	/// @param[in]		agentsIdx		The list of the indexes of the agents we want to update.
	/// @param[in]		nbIdx			Number of agents to work on.
	/// @param[in]		m_agentAnims	The animations for each agent.
	void triggerOffMeshConnections(dtCrowdAgent** agents, int* agentsIdx, const int nbIdx, dtCrowdAgentAnimation* m_agentAnims);

	/// Submits a new move request for the specified agent.
	/// Sets a flag indicate that the path of the agent is being replanned.
	///
	/// @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @param[in]		vel		The movement velocity. [(x, y, z)]
	///
	/// @return True if the request was successfully submitted.
	bool requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos);

	/// Moves an agent into a list according to the last time since its target was replanned.
	///
	/// @param[in]		newag			Agent we want to move.
	/// @param[in]		agents			The list of agents.
	/// @param[in]		nagents			Size of the list.
	/// @param[in]		maxAgents		Maximal number of agents.
	int addToPathQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents);

	/// Moves an agent into a list according to the last time since its path corridor was updated.
	///
	/// @param[in]		newag			Agent we want to move.
	/// @param[in]		agents			The list of agents.
	/// @param[in]		nagents			Size of the list.
	/// @param[in]		maxAgents		Maximal number of agents.
	int addToOptQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents);

	/// Is the agent standing over an offmesh connection?
	///
	/// @param[in]		ag			The agent.
	/// @param[in]		radius		The radius of the agent.
	///
	/// @return Returns true if the agent is standing over an offmesh connection.
	bool overOffmeshConnection(const dtCrowdAgent* ag, const float radius);

	/// Computes the distance from a given agent to its goal.
	///
	/// @param[in]		ag			The agent.
	/// @param[in]		range		The range of the agent (usually radius * 2).
	///
	/// @return Returns  the distance from the agent to its goal.
	float getDistanceToGoal(const dtCrowdAgent* ag, const float range);

	/// Computes the direction heading for the next corner target in a smooth way (as opposed to straight).
	///
	/// @param[in]		ag			The agent.
	/// @param[out]		dir			The direction resulting of the computation.
	void calcSmoothSteerDirection(const dtCrowdAgent* ag, float* dir);

	/// Computes the direction heading for the next corner target.
	///
	/// @param[in]		ag			The agent.
	/// @param[out]		dir			The direction resulting of the computation.
	void calcStraightSteerDirection(const dtCrowdAgent* ag, float* dir);

	dtQueryFilter m_filter;			///< Defines polygon filtering and traversal costs for navigation mesh query operations.
	dtPathQueue m_pathQueue;		///< A Queue of destination in order to reach the target.
	dtNavMeshQuery* m_navMeshQuery;	///< Used to perform queries on the navigation mesh.
	float m_ext[3];					///< The search distance along each axis.

	dtCrowdAgent* m_agents;		///< the list of agents (active of not) dealt with.
	dtPolyRef* m_pathResult;	///< The path results
	int m_maxAgents;			///< Maximal number of agents.
	int m_maxPathRes;			///< Maximal number of path results

	const int m_maxCommonNodes;		///< Maximal number of common nodes.
	const int m_maxPathQueueNodes;	///< Maximal number of nodes in the path queue.
	const int m_maxIterPerUpdate;	///< Maximal number of iterations per update.
};

#endif