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
		if (fabsf(diff[1]) >= (height+ag->height)/2.0f)
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
	m_crowdQuery(0),
	m_agentsEnv(0),
	m_maxAgents(0),
	m_nbActiveAgents(0),
	m_agents(0),
	m_activeAgents(0),
	m_agentsToUpdate(0),
	m_maxAgentRadius(0),
	m_maxPathQueueNodes(4096),
	m_maxCommonNodes(512)
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

	for (int i = 0; i < m_maxAgents; ++i)
		m_agentsEnv[i].~dtCrowdAgentEnvironment();

	dtFree(m_agents);
	m_agents = 0;
	m_maxAgents = 0;
	m_nbActiveAgents = 0;
	
	dtFree(m_activeAgents);
	m_activeAgents = 0;

	dtFree(m_agentsToUpdate);
	m_agentsToUpdate = 0;

	dtFree(m_crowdQuery);
	m_crowdQuery = 0;

	dtFree(m_agentsEnv);
	m_agentsEnv = 0;
}

/// @par
///
/// May be called more than once to purge and re-initialize the crowd.
bool dtCrowd::init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav)
{
	purge();

	// Creation of the crowd query
	void* mem = (dtCrowdQuery*) dtAlloc(sizeof(dtCrowdQuery), DT_ALLOC_PERM);
	if (!mem)
		return false;

	m_crowdQuery = new(mem) dtCrowdQuery(maxAgents);

	m_agentsEnv = (dtCrowdAgentEnvironment*) dtAlloc(sizeof(dtCrowdAgentEnvironment) * maxAgents, DT_ALLOC_PERM);
	if (!m_agentsEnv)
		return false;

	for (int i = 0; i < maxAgents; ++i)
		new(&m_agentsEnv[i]) dtCrowdAgentEnvironment(maxAgents);

	if (dtStatusFailed(m_crowdQuery->getNavMeshQuery()->init(nav, m_maxCommonNodes)))
		return false;
			
	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;
	m_nbActiveAgents = 0;
	m_agentsToUpdate = (int*) dtAlloc(sizeof(int) * maxAgents, DT_ALLOC_PERM);

	if (!m_agentsToUpdate)
		return false;

	for (int i = 0; i < m_maxAgents; ++i)
		m_agentsToUpdate[i] = i;
	
	if (!m_crowdQuery->getProximityGrid())
		return false;
	if (!m_crowdQuery->getProximityGrid()->init(m_maxAgents*4, maxAgentRadius*3))
		return false;
	
	m_agents = (dtCrowdAgent*)dtAlloc(sizeof(dtCrowdAgent) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_agents)
		return false;
	
	m_activeAgents = (dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_activeAgents)
		return false;

	if (!m_crowdQuery->getAgentsAnims())
		return false;
	
	for (int i = 0; i < m_maxAgents; ++i)
	{
		new(&m_agents[i]) dtCrowdAgent();
		m_agents[i].active = 0;
		m_agents[i].id = i;
		m_agents[i].behavior = 0;
		m_agents[i].userData = 0;
	}

	for (int i = 0; i < m_maxAgents; ++i)
		m_crowdQuery->getAgentsAnims()[i].active = 0;
	
	m_crowdQuery->getQueryExtents()[0] = m_maxAgentRadius * 2.0f;
	m_crowdQuery->getQueryExtents()[1] = m_maxAgentRadius * 1.5f;
	m_crowdQuery->getQueryExtents()[2] = m_maxAgentRadius * 2.0f;
	
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
	if (idx >= 0 && idx < m_maxAgents)
		return &m_agents[idx];

	return 0;
}

const dtCrowdAgentEnvironment* dtCrowd::getAgentsEnvironment() const
{
	return m_agentsEnv;
}

/// @par
/// 
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
dtCrowdAgent* dtCrowd::getAgent(const int idx)
{
	return &m_agents[idx];
}

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
int dtCrowd::addAgent(const float* pos)
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
	m_crowdQuery->getNavMeshQuery()->findNearestPoly(pos, m_crowdQuery->getQueryExtents(), 
												 m_crowdQuery->getQueryFilter(), &ref, nearest);
	
	ag->desiredSpeed = 0;

	dtVset(ag->dvel, 0,0,0);
	dtVset(ag->vel, 0,0,0);
	dtVcopy(ag->npos, nearest);
	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;
		
	ag->active = 1;
	
	return idx;
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

void dtCrowd::updateVelocity(const float dt, int* agentsIdx, int nbIdx)
{
	dtCrowdAgent** agents = m_activeAgents;
	getAllAgents(agents);
	nbIdx = (nbIdx < m_maxAgents) ? nbIdx : m_maxAgents;
	
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
		
		if (ag->behavior)
			ag->behavior->update(ag, ag, dt);
	}

	// Fake dynamic constraint
	for (int i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		const float maxDelta = ag->maxAcceleration * dt;
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

	// The current start position of the agent (not yet modified)
	dtPolyRef* currentPosPoly = (dtPolyRef*) dtAlloc(sizeof(dtPolyRef) * nbIdx, DT_ALLOC_TEMP);
	float* currentPos = (float*) dtAlloc(sizeof(float) * 3 * nbIdx, DT_ALLOC_TEMP);

	for (int i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		m_crowdQuery->getNavMeshQuery()->findNearestPoly(ag->npos, m_crowdQuery->getQueryExtents(), m_crowdQuery->getQueryFilter(), currentPosPoly + i, currentPos + (i * 3));
	}

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

			for (int j = 0; j < m_agentsEnv[ag->id].nbNeighbors; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[m_agentsEnv[ag->id].neighbors[j].idx];
				const int idx1 = getAgentIndex(nei);

				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;

				float dist = dtVlenSqr(diff);

				if (dist > dtSqr(ag->radius + nei->radius))
					continue;

				dist = sqrtf(dist);
				float pen = (ag->radius + nei->radius) - dist;
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
		float newPos[3];
		dtPolyRef visited[dtPathCorridor::MAX_VISITED];
		int visitedCount;
		m_crowdQuery->getNavMeshQuery()->moveAlongSurface(currentPosPoly[i], currentPos + (i * 3), ag->npos, m_crowdQuery->getQueryFilter(), newPos, 
													  visited, &visitedCount, dtPathCorridor::MAX_VISITED);

		// Get valid constrained position back.
		float newHeight = *(currentPos + (i * 3) + 1);
		m_crowdQuery->getNavMeshQuery()->getPolyHeight(currentPosPoly[i], newPos, &newHeight);
		newPos[1] = newHeight;

		dtVcopy(ag->npos, newPos);
	}

	// Update agents using off-mesh connection.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgentAnimation* anim = &m_crowdQuery->getAgentsAnims()[i];
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

	dtFree(currentPosPoly);
	dtFree(currentPos);
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
	m_crowdQuery->getProximityGrid()->clear();

	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent& ag = m_agents[i];

		const float* p = ag.npos;
		const float r = ag.radius;
		m_crowdQuery->getProximityGrid()->addItem((unsigned short)i, p[0]-r, p[2]-r, p[0]+r, p[2]+r);
	}

	// Get nearby navmesh segments and agents to collide with
	for (unsigned i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			m_agentsEnv[ag->id].boundary.reset();

		// Update the collision boundary after certain distance has been passed or
		// if it has become invalid.
		const float updateThr = ag->collisionQueryRange * 0.25f;
		if (dtVdist2DSqr(ag->npos, m_agentsEnv[ag->id].boundary.getCenter()) > dtSqr(updateThr) ||
			!m_agentsEnv[ag->id].boundary.isValid(m_crowdQuery->getNavMeshQuery(), m_crowdQuery->getQueryFilter()))
		{
			dtPolyRef ref;
			float nearest[3];
			m_crowdQuery->getNavMeshQuery()->findNearestPoly(ag->npos, m_crowdQuery->getQueryExtents(), m_crowdQuery->getQueryFilter(), &ref, nearest);

			m_agentsEnv[ag->id].boundary.update(ref, ag->npos, ag->collisionQueryRange, 
				m_crowdQuery->getNavMeshQuery(), m_crowdQuery->getQueryFilter());
		}
		// Query neighbour agents
		m_agentsEnv[ag->id].nbNeighbors = getNeighbours(ag->npos, ag->height, ag->collisionQueryRange,
			ag, m_agentsEnv[ag->id].neighbors, DT_CROWDAGENT_MAX_NEIGHBOURS,
			m_agents, m_nbActiveAgents, m_crowdQuery->getProximityGrid());

		for (int j = 0; j < m_agentsEnv[ag->id].nbNeighbors; j++)
			m_agentsEnv[ag->id].neighbors[j].idx = getAgentIndex(&m_agents[m_agentsEnv[ag->id].neighbors[j].idx]);
	}
}
	
void dtCrowd::update(const float dt, int* indexList, int nbIndex)
{
	updateEnvironment(indexList, nbIndex);
	updateVelocity(dt, indexList, nbIndex);
	updatePosition(dt, indexList, nbIndex);
}

bool dtCrowd::updateAgentPosition(int index, const float* position)
{
	if (index >= 0 && index < m_maxAgents)
	{
		dtCrowdAgent& ag = m_agents[index];
		dtPolyRef ref = 0;
		float nearestPosition[] = {0, 0, 0};

		if (dtStatusFailed(m_crowdQuery->getNavMeshQuery()->findNearestPoly(position, m_crowdQuery->getQueryExtents(), m_crowdQuery->getQueryFilter(), &ref, nearestPosition)))
			return false;

		// If no polygons have been found, it's a failure
		if (ref == 0)
			return false;
		
		dtVset(ag.dvel, 0, 0, 0);
		dtVset(ag.vel, 0, 0, 0);
		dtVcopy(ag.npos, nearestPosition);

		ag.state = DT_CROWDAGENT_STATE_WALKING;
		ag.desiredSpeed = 0;
		
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

dtCrowdQuery::~dtCrowdQuery()
{
	dtFree(m_agentAnims);
	m_agentAnims = 0;

	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFreeNavMeshQuery(m_navMeshQuery);
	m_navMeshQuery = 0;
}

dtCrowdQuery::dtCrowdQuery(int maxAgents)
	: m_maxAgents(maxAgents)
{
	m_navMeshQuery = dtAllocNavMeshQuery();
	m_grid = dtAllocProximityGrid();
	m_agentAnims = (dtCrowdAgentAnimation*) dtAlloc(sizeof(dtCrowdAgentAnimation) * maxAgents, DT_ALLOC_PERM);
}

dtCrowdAgentAnimation* dtCrowdQuery::getAgentsAnims() 
{ 
	return m_agentAnims; 
}

float* dtCrowdQuery::getQueryExtents() 
{ 
	return m_ext; 
}

dtNavMeshQuery* dtCrowdQuery::getNavMeshQuery() 
{ 
	return m_navMeshQuery; 
}

dtQueryFilter* dtCrowdQuery::getQueryFilter() 
{ 
	return &m_filter; 
}

dtProximityGrid* dtCrowdQuery::getProximityGrid() 
{ 
	return m_grid; 
}

dtCrowdAgentEnvironment::dtCrowdAgentEnvironment(int nbMaxAgents) 
	: nbNeighbors(0) 
{
	boundary.reset();
}

dtCrowdAgentEnvironment::~dtCrowdAgentEnvironment()
{
}
