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

#include "DetourPathCorridor.h"
#include "DetourPathQueue.h"
#include "DetourNavMeshQuery.h"
#include "DetourParametrizedBehavior.h"

class dtNavMesh;
struct dtCrowdAgent;
struct dtCrowdAgentDebugInfo;
struct dtCrowdAgentEnvironment;
class dtCrowdQuery;

template <typename T>
class dtParametrizedBehavior;


/// The state in which the move request of an agent is.
enum MoveRequestState
{
	DT_CROWDAGENT_TARGET_NONE = 0,			///< The agent does not have a target
	DT_CROWDAGENT_TARGET_FAILED,			///< The agent's target affectation failed
	DT_CROWDAGENT_TARGET_VALID,				///< The agent's target is valid
	DT_CROWDAGENT_TARGET_REQUESTING,		///< The agent's target is currently requesting a target
	DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,	///< The path to reach teh target is potentially unreachable
	DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,	///< The agent is waiting for a path in order to reach its target
	DT_CROWDAGENT_TARGET_VELOCITY,			///< The agent's target is a veloicty, not a position
};

/// This class gives informations about the current path of the agent
struct dtCrowdAgentDebugInfo
{
	unsigned idx;						///< Index of  the agent
	float optStart[3], optEnd[3];		///< Begin and end of the path
	dtObstacleAvoidanceDebugData* vod;	///< Informations about how the agent avoids obstacles
};

/// Parameters for the path following behavior
/// @warning Before using this parameter, make sure to call the `dtPathFollowingParams::init()` method and 
/// `dtPathFollowingParams::preparePath()` (in that order).
/// @ingroup behavior
struct dtPathFollowingParams
{
	dtPathFollowingParams();

	dtCrowdAgentDebugInfo* debugInfos;	///< A debug object to load with debug information. [Opt]
	unsigned debugIndex;				///< The index of the agent for debug purpose.
	float pathOptimizationRange;		///< The path visibility optimization range. [Limit: > 0]
	float targetReplanTime;				/// <Time since the agent's target was replanned.
	dtPolyRef targetRef;				///< Target polyref of the movement request.
	bool targetReplan;					///< Flag indicating that the current path is being replanned.
	dtPathQueueRef targetPathqRef;		///< Path finder ref.
	float targetPos[3];					///< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	unsigned char targetState;			///< State of the movement request.
	unsigned ncorners;					/// The number of corners.
	
	/// The maximum number of corners a crowd agent will look ahead in the path.
	/// This value is used for sizing the crowd agent corner buffers.
	/// Due to the behavior of the crowd manager, the actual number of useful
	/// corners will be one less than this number.
	/// @ingroup behavior
	static const unsigned DT_CROWDAGENT_MAX_CORNERS = 4;

	float cornerVerts[DT_CROWDAGENT_MAX_CORNERS*3]; ///< The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]

	///< The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	unsigned char cornerFlags[DT_CROWDAGENT_MAX_CORNERS];

	///< The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	dtPolyRef cornerPolys[DT_CROWDAGENT_MAX_CORNERS];

	///< Time since the agent's path corridor was optimized.
	float topologyOptTime;

	///< The path corridor the agent is using.
	dtPathCorridor corridor;

	/// Initializes the corridor.
	/// This method MUST be called before the method `dtPathFollowingParams::preparePath()`.
	/// @param[in]	maxPathResult	The maximum number of polygons that can be stored in a corridor.
	/// @return	True if the initialization was successful, false otherwise
	bool init(unsigned maxPathResults);

	/// Configure the corridor that is used to represent the path.
	/// This creates the first polygon of the corridor according to the position of the agent
	/// This method MUST be called before using the path following behavior with this parameter (but call `dtPathFollowingParams::init()` first).
	/// @param[in]	position	The position of the agent. Used to determine which polygon it is on.
	/// @param[in]	query		Used to access the navigation mesh query in order to get the polygon the agent is on.
	bool preparePath(const float* position, const dtCrowdQuery& query);
};

/// Defines a behavior for pathfollowing.
///
/// Using a navigation mesh, the pathfollowing behavior works on 
/// a list of agent in order to update their velocity so they can
/// reach their goal.
/// @ingroup behavior
class dtPathFollowing : public dtParametrizedBehavior<dtPathFollowingParams>
{
public:
	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	///
	/// @return		A pointer on a newly allocated behavior
	static dtPathFollowing* allocate(unsigned nbMaxAgents);

	/// Frees the given behavior
	///
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtPathFollowing* ptr);

	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	dtPathFollowing(unsigned nbMaxAgents);
	~dtPathFollowing();

	/// Initializes the behavior.
	///
	/// Must be called before using the behavior.
	/// @param[in]		crowdQuery	An object granting access to several elements of the crowd (animations, navigation mesh queries, etc.)
	/// @param[in]		maxPathRes	Maximum number of polygons for a path.
	///
	/// @return True if the initialization succeeded, false otherwise.
	bool init(dtCrowdQuery& crowdQuery, unsigned maxPathRes = 256);

	/// Submits a new move request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		ref		The position's polygon reference.
	///  @param[in]		pos		The position within the polygon. [(x, y, z)]
	/// @return True if the request was successfully submitted.
	bool requestMoveTarget(const unsigned idx, dtPolyRef ref, const float* pos);
	
	/// Resets any request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return True if the request was successfully reseted.
	bool resetMoveTarget(const unsigned idx);

	/// Cleans the class before destroying
	void purge();

	/// @name Gets the path results
	/// @{
	const dtPolyRef* getPathRes() const { return m_pathResult; }
	dtPolyRef* getPathRes() { return m_pathResult; }
	/// @}

private:
    dtPathFollowing(const dtPathFollowing&);
    dtPathFollowing& operator=(const dtPathFollowing&);

	/// Checks that the given agents still have valid paths.
	/// 
	/// @param[in]		ag				The agent to work on.
	/// @param[in]		dt				The time, in seconds, to update the simulation. [Limit: > 0]
	void checkPathValidity(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, const float dt, dtPathFollowingParams* agParams);

	/// Update async move request and path finder.
	void updateMoveRequest(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
		dtPathFollowingParams& newParams);

	virtual void doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
		const dtPathFollowingParams& currentParams, dtPathFollowingParams& newParams, float dt);

	/// Optimize path topology.
	/// 
	/// @param[in]		ag			The agent to work on.
	/// @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void updateTopologyOptimization(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& ag, const float dt, dtPathFollowingParams* agParams);

	/// Performs some checking and optimization on the agent.
	///
	/// @param[in]		ag			The agent to work on.
	/// @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void prepare(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt,dtPathFollowingParams& newParam);

	/// Computes the new velocity of the old agent according to its parameters, and puts the result into the new agent.
	///
	/// @param[in]		oldAgent	The agent whose velocity must be updated.
	/// @param[out]		newAgent	The agent storing the new parameters.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void getVelocity(const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, dtPathFollowingParams& agParams);

	/// Finds the next corner the agent should aim to
	/// 
	/// @param[in]		ag			The agent to work on.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void getNextCorner(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& ag, dtPathFollowingParams& agParams);

	/// Checks whether the agent is on an offmesh connection. If so, the path is adjusted.
	/// 
	/// @param[in]		agents			List of active agents.
	/// @param[out]		newAgent		The agent storing the new parameters.
	/// @param[in]		agParams		The parameters of the agent for this behavior
	void triggerOffMeshConnections(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, dtPathFollowingParams* agParams);

	/// Submits a new move request for the specified agent.
	/// Sets a flag indicate that the path of the agent is being replanned.
	///
	/// @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @param[in]		ref		A reference to the destination polygon
	/// @param[in]		pos		The destination
	///
	/// @return True if the request was successfully submitted.
	bool requestMoveTargetReplan(const unsigned idx, dtPolyRef ref, const float* pos);

	/// Moves an agent into a list according to the last time since its target was replanned.
	///
	/// @param[in]		newag			Agent we want to move.
	/// @param[in]		agents			The list of agents.
	/// @param[in]		nagents			Size of the list.
	/// @param[in]		maxAgents		Maximal number of agents.
	int addToPathQueue(const dtCrowdAgent& newag, dtCrowdAgent** agents, const unsigned nagents, const unsigned maxAgents);

	/// Moves an agent into a list according to the last time since its path corridor was updated.
	///
	/// @param[in]		newag			Agent we want to move.
	/// @param[in]		agents			The list of agents.
	/// @param[in]		nagents			Size of the list.
	/// @param[in]		maxAgents		Maximal number of agents.
	int addToOptQueue(const dtCrowdAgent& newag, dtCrowdAgent** agents, const unsigned nagents, const unsigned maxAgents);

	/// Is the agent standing over an offmesh connection?
	///
	/// @param[in]		ag			The agent.
	/// @param[in]		radius		The radius of the agent.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	///
	/// @return Returns true if the agent is standing over an offmesh connection.
	bool overOffmeshConnection(const dtCrowdAgent& ag, const float radius, dtPathFollowingParams* agParams);

	/// Computes the distance from a given agent to its goal.
	///
	/// @param[in]		ag			The agent.
	/// @param[in]		range		The range of the agent (usually radius * 2).
	/// @param[in]		agParams	The parameters of the agent for this behavior
	///
	/// @return Returns  the distance from the agent to its goal.
	float getDistanceToGoal(const dtCrowdAgent& ag, const float range, dtPathFollowingParams* agParams);

	/// Computes the direction heading for the next corner target in a smooth way (as opposed to straight).
	///
	/// @param[in]		ag			The agent.
	/// @param[out]		dir			The direction resulting of the computation.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void calcSmoothSteerDirection(const dtCrowdAgent& ag, float* dir, dtPathFollowingParams* agParams);

	/// Computes the direction heading for the next corner target.
	///
	/// @param[in]		ag			The agent.
	/// @param[out]		dir			The direction resulting of the computation.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void calcStraightSteerDirection(const dtCrowdAgent& ag, float* dir, dtPathFollowingParams* agParams);

	dtPathQueue m_pathQueue;				///< A Queue of destination in order to reach the target.

	dtPolyRef* m_pathResult;				///< The path results
	unsigned m_maxAgents;					///< Maximal number of agents.
	unsigned m_maxPathRes;					///< Maximal number of path results

	const unsigned m_maxCommonNodes;		///< Maximal number of common nodes.
	const unsigned m_maxPathQueueNodes;		///< Maximal number of nodes in the path queue.
	const unsigned m_maxIterPerUpdate;		///< Maximal number of iterations per update.
};

#endif
