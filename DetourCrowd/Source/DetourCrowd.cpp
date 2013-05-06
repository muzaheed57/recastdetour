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

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>
#include <new>

#include "DetourAlignmentBehavior.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourBehavior.h"
#include "DetourCohesionBehavior.h"
#include "DetourCollisionAvoidance.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourFlockingBehavior.h"
#include "DetourGoToBehavior.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourPathFollowing.h"
#include "DetourSeekBehavior.h"
#include "DetourSeparationBehavior.h"


/// Constant used for float comparisons.
static const float EPSILON = 0.0001f;

dtCrowd* dtAllocCrowd()
{
	void* mem = dtAlloc(sizeof(dtCrowd), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtCrowd;
}

void dtFreeCrowd(dtCrowd* ptr)
{
	if (!ptr) return;
	ptr->~dtCrowd();
	dtFree(ptr);
}


static const int MAX_ITERS_PER_UPDATE = 100;


inline float tween(const float t, const float t0, const float t1)
{
	return dtClamp((t-t0) / (t1-t0), 0.0f, 1.0f);
}

static void integrate(dtCrowdAgent* ag, const float dt)
{
	if (dtVlen(ag->vel) > EPSILON)
		dtVmad(ag->npos, ag->npos, ag->vel, dt);
	else
		dtVset(ag->vel,0,0,0);
}

static int addNeighbour(const int idx, const float dist,
						dtCrowdNeighbour* neis, const int nneis, const int maxNeis)
{
	// Insert neighbour based on the distance.
	dtCrowdNeighbour* nei = 0;
	if (!nneis)
	{
		nei = &neis[nneis];
	}
	else if (dist >= neis[nneis-1].dist)
	{
		if (nneis >= maxNeis)
			return nneis;
		nei = &neis[nneis];
	}
	else
	{
		int i;
		for (i = 0; i < nneis; ++i)
			if (dist <= neis[i].dist)
				break;
		
		const int tgt = i+1;
		const int n = dtMin(nneis-i, maxNeis-tgt);
		
		dtAssert(tgt+n <= maxNeis);
		
		if (n > 0)
			memmove(&neis[tgt], &neis[i], sizeof(dtCrowdNeighbour)*n);
		nei = &neis[i];
	}
	
	memset(nei, 0, sizeof(dtCrowdNeighbour));
	
	nei->idx = idx;
	nei->dist = dist;
	
	return dtMin(nneis+1, maxNeis);
}

static int getNeighbours(const float* pos, const float height, const float range,
						 const dtCrowdAgent* skip, dtCrowdNeighbour* result, const int maxResult,
						 dtCrowdAgent* agents, const int /*nagents*/, dtProximityGrid* grid)
{
	int n = 0;
	
	static const int MAX_NEIS = 32;
	unsigned short ids[MAX_NEIS];
	int nids = grid->queryItems(pos[0]-range, pos[2]-range,
								pos[0]+range, pos[2]+range,
								ids, MAX_NEIS);
	
	for (int i = 0; i < nids; ++i)
	{
		const dtCrowdAgent* ag = &agents[ids[i]];

		if (!ag->active)
			continue;
		
		if (ag == skip) continue;
		
		// Check for overlap.
		float diff[3];
		dtVsub(diff, pos, ag->npos);
		if (fabsf(diff[1]) >= (height+ag->params.height)/2.0f)
			continue;
		diff[1] = 0;
		const float distSqr = dtVlenSqr(diff);
		if (distSqr > dtSqr(range))
			continue;
		
		n = addNeighbour(ids[i], distSqr, result, n, maxResult);
	}
	return n;
}


/**
@class dtCrowd
@par

This is the core class of the @ref crowd module.  See the @ref crowd documentation for a summary
of the crowd features.

A common method for setting up the crowd is as follows:

-# Allocate the crowd using #dtAllocCrowd.
-# Initialize the crowd using #init().
-# Set the avoidance configurations using #setObstacleAvoidanceParams().
-# Add agents using #addAgent() and make an initial movement request using #requestMoveTarget().

A common process for managing the crowd is as follows:

-# Call #update() to allow the crowd to manage its agents.
-# Retrieve agent information using #getActiveAgents().
-# Make movement requests using #requestMoveTarget() when movement goal changes.
-# Repeat every frame.

Some agent configuration settings can be updated using #updateAgentParameters().  But the crowd owns the
agent position.  So it is not possible to update an active agent's position.  If agent position
must be fed back into the crowd, the agent must be removed and re-added.

Notes: 

- Path related information is available for newly added agents only after an #update() has been
  performed.
- Agent objects are kept in a pool and re-used.  So it is important when using agent objects to check the value of
  #dtCrowdAgent::active to determine if the agent is actually in use or not.
- This class is meant to provide 'local' movement. There is a limit of 256 polygons in the path corridor.  
  So it is not meant to provide automatic pathfinding services over long distances.

@see dtAllocCrowd(), dtFreeCrowd(), init(), dtCrowdAgent

*/

static const int MAX_AVOIDANCE_PARAMS = 4;

dtCrowd::dtCrowd() :
	m_maxAgents(0),
	m_nbActiveAgents(0),
	m_agents(0),
	m_activeAgents(0),
	m_agentAnims(0),
	m_agentsToUpdate(0),
	m_grid(0),
	m_maxPathResult(256),
	m_maxAgentRadius(0),
	m_maxPathQueueNodes(4096),
	m_maxCommonNodes(512),
	m_navMeshQuery(0)
{
}

dtCrowd::~dtCrowd()
{
	purge();
}

void dtCrowd::purge()
{
	for (int i = 0; i < m_maxAgents; ++i)
		m_agents[i].~dtCrowdAgent();

	dtFree(m_agents);
	m_agents = 0;
	m_maxAgents = 0;
	m_nbActiveAgents = 0;
	
	dtFree(m_activeAgents);
	m_activeAgents = 0;

	dtFree(m_agentAnims);
	m_agentAnims = 0;
	
	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFree(m_agentsToUpdate);
	m_agentsToUpdate = 0;

	dtFreeNavMeshQuery(m_navMeshQuery);
	m_navMeshQuery = 0;
}

/// @par
///
/// May be called more than once to purge and re-initialize the crowd.
bool dtCrowd::init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav)
{
	purge();

	m_navMeshQuery = dtAllocNavMeshQuery();

	if (dtStatusFailed(m_navMeshQuery->init(nav, m_maxCommonNodes)))
		return false;

	if (!m_pathQueue.init(m_maxPathResult, m_maxPathQueueNodes, nav))
		return false;
	
	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;
	m_nbActiveAgents = 0;
	m_agentsToUpdate = (int*) dtAlloc(sizeof(int) * maxAgents, DT_ALLOC_PERM);

	if (!m_agentsToUpdate)
		return false;

	for (int i = 0; i < m_maxAgents; ++i)
		m_agentsToUpdate[i] = i;

	m_grid = dtAllocProximityGrid();

	if (!m_grid)
		return false;
	if (!m_grid->init(m_maxAgents*4, maxAgentRadius*3))
		return false;
	
	m_agents = (dtCrowdAgent*)dtAlloc(sizeof(dtCrowdAgent)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_agents)
		return false;
	
	m_activeAgents = (dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_activeAgents)
		return false;

	m_agentAnims = (dtCrowdAgentAnimation*)dtAlloc(sizeof(dtCrowdAgentAnimation)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_agentAnims)
		return false;
	
	for (int i = 0; i < m_maxAgents; ++i)
	{
		new(&m_agents[i]) dtCrowdAgent();
		m_agents[i].active = 0;

		if (!m_agents[i].corridor.init(m_maxPathResult))
			return false;
	}

	for (int i = 0; i < m_maxAgents; ++i)
		m_agentAnims[i].active = 0;
	
	m_ext[0] = m_maxAgentRadius * 2.0f;
	m_ext[1] = m_maxAgentRadius * 1.5f;
	m_ext[2] = m_maxAgentRadius * 2.0f;
	
	return true;
}

const int dtCrowd::getAgentCount() const
{
	return m_maxAgents;
}

/// @par
/// 
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
const dtCrowdAgent* dtCrowd::getAgent(const int idx) const
{
	if (idx < 0 || idx > m_maxAgents)
		return 0;
	return &m_agents[idx];
}

/// @par
/// 
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
dtCrowdAgent* dtCrowd::getAgent(const int idx)
{
	return &m_agents[idx];
}

void dtCrowd::updateAgentParameters(const int idx, const dtCrowdAgentParams* params)
{
	if (idx < 0 || idx > m_maxAgents)
		return;
	memcpy(&m_agents[idx].params, params, sizeof(dtCrowdAgentParams));
}

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
int dtCrowd::addAgent(const float* pos, const dtCrowdAgentParams* params)
{
	// Find empty slot.
	int idx = -1;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			break;
		}
	}
	if (idx == -1)
		return -1;
	
	dtCrowdAgent* ag = &m_agents[idx];

	// Find nearest position on navmesh and place the agent there.
	float nearest[3];
	dtPolyRef ref;
	getNavMeshQuery()->findNearestPoly(pos, getQueryExtents(), getEditableFilter(), &ref, nearest);
	
	ag->corridor.reset(ref, nearest);
	ag->boundary.reset();

	updateAgentParameters(idx, params);
	
	ag->topologyOptTime = 0;
	ag->targetReplanTime = 0;
	ag->nneis = 0;
	ag->ncorners = 0;

	dtVset(ag->dvel, 0,0,0);
	dtVset(ag->vel, 0,0,0);
	dtVcopy(ag->npos, nearest);

	ag->desiredSpeed = 0;
	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;

	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	
	ag->active = 1;
	++m_nbActiveAgents;

	return idx;
}

const dtQueryFilter* dtCrowd::getFilter() const 
{ 
	return &m_filter; 
}

dtQueryFilter* dtCrowd::getEditableFilter() 
{ 
	return &m_filter; 
}

const float* dtCrowd::getQueryExtents() const 
{ 
	return m_ext;//->getQueryExtent(); 
}

const dtPathQueue* dtCrowd::getPathQueue() const 
{ 
	return &m_pathQueue; 
}

dtPathQueue* dtCrowd::getPathQueue()
{ 
	return &m_pathQueue; 
}

const dtNavMeshQuery* dtCrowd::getNavMeshQuery() const 
{ 
	return m_navMeshQuery; 
}

dtNavMeshQuery* dtCrowd::getNavMeshQuery() 
{ 
	return m_navMeshQuery;
}

/// @par
///
/// The agent is deactivated and will no longer be processed. Its #dtCrowdAgent object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
void dtCrowd::removeAgent(const int idx)
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		if (m_agents[idx].active != 0)
		{
			m_agents[idx].active = 0;
			--m_nbActiveAgents;
		}
	}
}

/// @par
/// 
/// This method is used when a new target is set.
/// 
/// The position will be constrained to the surface of the navigation mesh.
///
/// The request will be processed during the next #update().
bool dtCrowd::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	if (!ref)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

bool dtCrowd::requestMoveVelocity(const int idx, const float* vel)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	
	dtCrowdAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = 0;
	dtVcopy(ag->targetPos, vel);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_VELOCITY;
	
	return true;
}

bool dtCrowd::resetMoveTarget(const int idx)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	
	dtCrowdAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = 0;
	dtVset(ag->targetPos, 0,0,0);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	
	return true;
}

int dtCrowd::getActiveAgents(dtCrowdAgent** agents, const int maxAgents)
{
	int n = 0;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active) continue;
		if (n < maxAgents)
			agents[n++] = &m_agents[i];
	}
	return n;
}

void dtCrowd::getAllAgents(dtCrowdAgent** agents)
{
	for (int i = 0; i < m_maxAgents; ++i)
		agents[i] = &m_agents[i];
}

void dtCrowd::updateVelocity(const float dt, dtCrowdAgentDebugInfo* debug, int* agentsIdx, int nbIdx)
{
	dtCrowdAgent** agents = m_activeAgents;
	getAllAgents(agents);
	nbIdx = (nbIdx < m_nbActiveAgents) ? nbIdx : m_nbActiveAgents;
	
	// If we want to update every agent
	if (agentsIdx == 0)
	{
		agentsIdx = m_agentsToUpdate;
		nbIdx = m_maxAgents;
	}

	for (int i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;
		
		if (ag->params.steeringBehavior)
			ag->params.steeringBehavior->update(ag, ag, dt);
	}

	// Fake dynamic constraint
	for (int i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		const float maxDelta = ag->params.maxAcceleration * dt;
		float dv[3];
		dtVsub(dv, ag->dvel, ag->vel);
		float ds = dtVlen(dv);

		if (ds > maxDelta)
			dtVscale(dv, dv, maxDelta/ds);

		dtVadd(ag->vel, ag->vel, dv);
	}
}

void dtCrowd::updatePosition(const float dt, int* agentsIdx, int nbIdx)
{
	// If we want to update every agent
	if (nbIdx == 0)
	{
		agentsIdx = m_agentsToUpdate;
		nbIdx = m_maxAgents;
	}
	
	nbIdx = (nbIdx < m_maxAgents) ? nbIdx : m_maxAgents;

	// Integrate.
	for (int i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
	}

	// Handle collisions.
	static const float COLLISION_RESOLVE_FACTOR = 0.7f;

	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nbIdx; ++i)
		{
			dtCrowdAgent* ag = 0;

			if (!getActiveAgent(&ag, agentsIdx[i]))
				continue;

			const int idx0 = getAgentIndex(ag);

			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVset(ag->disp, 0,0,0);

			float w = 0;

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				const int idx1 = getAgentIndex(nei);

				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;

				float dist = dtVlenSqr(diff);

				if (dist > dtSqr(ag->params.radius + nei->params.radius))
					continue;

				dist = sqrtf(dist);
				float pen = (ag->params.radius + nei->params.radius) - dist;
				if (dist < EPSILON)
				{
					// Agents on top of each other, try to choose diverging separation directions.
					if (idx0 > idx1)
						dtVset(diff, -ag->dvel[2], 0, ag->dvel[0]);
					else
						dtVset(diff, ag->dvel[2], 0, -ag->dvel[0]);
					pen = 0.01f;
				}
				else
				{
					pen = (1.0f / dist) * (pen * 0.5f) * COLLISION_RESOLVE_FACTOR;
				}

				dtVmad(ag->disp, ag->disp, diff, pen);			

				w += 1.0f;
			}

			if (w > EPSILON)
			{
				const float iw = 1.0f / w;
				dtVscale(ag->disp, ag->disp, iw);
			}
		}
		
		for (int i = 0; i < nbIdx; ++i)
		{
			dtCrowdAgent* ag = 0;

			if (!getActiveAgent(&ag, agentsIdx[i]))
				continue;

			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}
	
	for (int i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Move along navmesh.
		ag->corridor.movePosition(ag->npos, getNavMeshQuery(), getEditableFilter());
		// Get valid constrained position back.
		dtVcopy(ag->npos, ag->corridor.getPos());

		// If not using path, truncate the corridor to just one poly.
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			ag->corridor.reset(ag->corridor.getFirstPoly(), ag->npos);
		}
	}

	// Update agents using off-mesh connection.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgentAnimation* anim = &m_agentAnims[i];
		if (!anim->active)
			continue;
		dtCrowdAgent* ag = &m_agents[i];

		anim->t += dt;
		if (anim->t > anim->tmax)
		{
			// Reset animation
			anim->active = 0;
			// Prepare agent for walking.
			ag->state = DT_CROWDAGENT_STATE_WALKING;
			continue;
		}

		// Update position
		const float ta = anim->tmax*0.15f;
		const float tb = anim->tmax;
		if (anim->t < ta)
		{
			const float u = tween(anim->t, 0.0, ta);
			dtVlerp(ag->npos, anim->initPos, anim->startPos, u);
		}
		else
		{
			const float u = tween(anim->t, ta, tb);
			dtVlerp(ag->npos, anim->startPos, anim->endPos, u);
		}

		// Update velocity.
		dtVset(ag->vel, 0,0,0);
		dtVset(ag->dvel, 0,0,0);
	}
}

void dtCrowd::updateEnvironment(int* agentsIdx, int nbIdx)
{
	// If we want to update every agent
	if (agentsIdx == 0)
	{
		agentsIdx = m_agentsToUpdate;
		nbIdx = m_maxAgents;
	}

	nbIdx = (nbIdx < m_maxAgents) ? nbIdx : m_maxAgents;	
	m_grid->clear();

	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent& ag = m_agents[i];

		const float* p = ag.npos;
		const float r = ag.params.radius;
		m_grid->addItem((unsigned short)i, p[0]-r, p[2]-r, p[0]+r, p[2]+r);
	}

	// Get nearby navmesh segments and agents to collide with
	for (unsigned i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Update the collision boundary after certain distance has been passed or
		// if it has become invalid.
		const float updateThr = ag->params.collisionQueryRange * 0.25f;
		if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(updateThr) ||
			!ag->boundary.isValid(m_navMeshQuery, &m_filter))
		{
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->params.collisionQueryRange,
				getNavMeshQuery(), getEditableFilter());
		}
		// Query neighbour agents
		ag->nneis = getNeighbours(ag->npos, ag->params.height, ag->params.collisionQueryRange,
			ag, ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS,
			m_agents, m_maxAgents, m_grid);
		for (int j = 0; j < ag->nneis; j++)
			ag->neis[j].idx = getAgentIndex(&m_agents[ag->neis[j].idx]);
	}
}
	
void dtCrowd::update(const float dt, dtCrowdAgentDebugInfo* debug, int* indexList, int nbIndex)
{
	updateEnvironment(indexList, nbIndex);
	updateVelocity(dt, debug, indexList, nbIndex);
	updatePosition(dt, indexList, nbIndex);
}

bool dtCrowd::updateAgentPosition(int index, const float* position)
{
	if (index >= 0 && index < m_maxAgents)
	{
		dtCrowdAgent& ag = m_agents[index];
		dtPolyRef ref = 0;
		float nearestPosition[] = {0, 0, 0};

		if (dtStatusFailed(getNavMeshQuery()->findNearestPoly(position, getQueryExtents(), getFilter(), &ref, nearestPosition)))
			return false;

		// If no polygons have been found, it's a failure
		if (ref == 0)
			return false;

		// Since the agent has moved, we reset it's corridor
		ag.corridor.reset(ref, nearestPosition);

		ag.topologyOptTime = 0;
		ag.nneis = 0;
		ag.ncorners = 0;
		ag.desiredSpeed = 0;

		dtVset(ag.dvel, 0, 0, 0);
		dtVset(ag.vel, 0, 0, 0);
		dtVcopy(ag.npos, nearestPosition);

		ag.state = DT_CROWDAGENT_STATE_WALKING;
		ag.targetState = DT_CROWDAGENT_TARGET_REQUESTING;
		ag.targetPathqRef = DT_PATHQ_INVALID;
		ag.targetReplan = false;
		
		return true;
	}

	return false;
}

bool dtCrowd::getActiveAgent(dtCrowdAgent** ag, int id)
{
	if (id >= 0 && id < m_maxAgents)
	{
		if (m_agents[id].active)
		{
			*ag = &m_agents[id];
			return true;
		}
	}

	return false;
}

bool dtCrowd::agentIsMoving( int index ) const
{
	return (m_agents[index].desiredSpeed > EPSILON && dtVlen(m_agents[index].vel) > EPSILON);
}
