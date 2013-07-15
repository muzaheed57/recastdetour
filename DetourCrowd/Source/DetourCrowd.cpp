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

#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourBehavior.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourPathFollowing.h"



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
	if (dtVlen(ag->velocity) > EPSILON)
		dtVmad(ag->position, ag->position, ag->velocity, dt);
	else
		dtVset(ag->velocity,0,0,0);
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
		dtVsub(diff, pos, ag->position);

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

A crowd handles every agent it contains. 
You can think of it as a big container for agents.

You can see `dtCrowd` as an interface between you and the agents it contains.
You cannot modify the agents inside the crowd directly, 
the interface will only grant you a constant access to the data.

Through `dtCrowd` you can do several things:

- Have access to the agents (although not directly)
- Have access to the data the crowd uses to update the agents (navigation mesh, proximity grid, etc.)
- Add / Remove / Edit agents inside the crowd
- Update the crowd. This will update every agents using their behaviors. When updating an agent you can 
update its position, velocity, environment or all of those.

For a more complete documentation, check the @ref crowd module.

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
	m_maxCommonNodes(512),
	m_disp(0),
	m_collisionQueryRange(4)
{
}

dtCrowd::~dtCrowd()
{
	purge();
}

void dtCrowd::purge()
{
	for (unsigned i = 0; i < m_maxAgents; ++i)
		m_agents[i].~dtCrowdAgent();

	for (unsigned i = 0; i < m_maxAgents; ++i)
		m_agentsEnv[i].~dtCrowdAgentEnvironment();

	if (m_disp)
	{
		for (unsigned i = 0; i < m_maxAgents; ++i)
			dtFree(m_disp[i]);

		dtFree(m_disp);
		m_disp = 0;
	}

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
bool dtCrowd::init(const unsigned maxAgents, const float maxAgentRadius, dtNavMesh* nav, float collisionRange)
{
	purge();

	m_disp = (float**) dtAlloc(sizeof(float*) * maxAgents, DT_ALLOC_PERM);
	m_collisionQueryRange = collisionRange;

	for (unsigned i = 0; i < maxAgents; ++i)
		m_disp[i] = (float*) dtAlloc(sizeof(float) * 3, DT_ALLOC_PERM);

	// Creation of the crowd query
	void* mem = (dtCrowdQuery*) dtAlloc(sizeof(dtCrowdQuery), DT_ALLOC_PERM);
	if (!mem)
		return false;
	
	m_agentsEnv = (dtCrowdAgentEnvironment*) dtAlloc(sizeof(dtCrowdAgentEnvironment) * maxAgents, DT_ALLOC_PERM);
	if (!m_agentsEnv)
		return false;

	for (unsigned i = 0; i < maxAgents; ++i)
		new(&m_agentsEnv[i]) dtCrowdAgentEnvironment();
				
	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;
	m_nbActiveAgents = 0;
	m_agentsToUpdate = (unsigned*) dtAlloc(sizeof(int) * maxAgents, DT_ALLOC_PERM);

	if (!m_agentsToUpdate)
		return false;

	for (unsigned i = 0; i < m_maxAgents; ++i)
		m_agentsToUpdate[i] = i;
		
	m_agents = (dtCrowdAgent*)dtAlloc(sizeof(dtCrowdAgent) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_agents)
		return false;

	m_crowdQuery = new(mem) dtCrowdQuery(maxAgents, m_agents, m_agentsEnv);

	if (dtStatusFailed(m_crowdQuery->getNavMeshQuery()->init(nav, m_maxCommonNodes)))
		return false;

	if (!m_crowdQuery->getProximityGrid())
		return false;

	if (!m_crowdQuery->getProximityGrid()->init(m_maxAgents*4, maxAgentRadius*3))
		return false;
	
	m_activeAgents = (dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_activeAgents)
		return false;
	
	for (unsigned i = 0; i < m_maxAgents; ++i)
	{
		new(&m_agents[i]) dtCrowdAgent();
		m_agents[i].active = 0;
		m_agents[i].id = i;
		m_agents[i].behavior = 0;
		m_agents[i].userData = 0;
	}
	
	m_crowdQuery->getQueryExtents()[0] = m_maxAgentRadius * 2.0f;
	m_crowdQuery->getQueryExtents()[1] = m_maxAgentRadius * 1.5f;
	m_crowdQuery->getQueryExtents()[2] = m_maxAgentRadius * 2.0f;
		
	return true;
}

const unsigned dtCrowd::getAgentCount() const
{
	return m_maxAgents;
}

/// @par
/// 
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
const dtCrowdAgent* dtCrowd::getAgent(const unsigned id) const
{
	return m_crowdQuery->getAgent(id);
}

void dtCrowd::fetchAgent(dtCrowdAgent& ag, unsigned id) const
{
	m_crowdQuery->fetchAgent(ag, id);
}

const dtCrowdAgentEnvironment* dtCrowd::getAgentEnvironment(unsigned id) const
{
	return m_crowdQuery->getAgentEnvironment(id);
}

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
bool dtCrowd::addAgent(dtCrowdAgent& agent, const float* pos)
{
	// Find empty slot.
	int idx = -1;
	for (unsigned i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			break;
		}
	}

	if (idx == -1)
		return false;
	
	dtCrowdAgent* ag = &m_agents[idx];

	// Find nearest position on navmesh and place the agent there.
	float nearest[3];
	dtPolyRef ref;
	m_crowdQuery->getNavMeshQuery()->findNearestPoly(pos, m_crowdQuery->getQueryExtents(), 
												 m_crowdQuery->getQueryFilter(), &ref, nearest);
	
	dtVset(ag->desiredVelocity, 0, 0, 0);
	dtVset(ag->velocity, 0, 0, 0);
	dtVcopy(ag->position, nearest);
	
	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;
		
	ag->active = 1;

	agent = *ag;
	

	return true;
}

/// @par
///
/// The agent is deactivated and will no longer be processed. Its #dtCrowdAgent object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
void dtCrowd::removeAgent(unsigned id)
{
	if (id < m_maxAgents)
	{
		if (m_agents[id].active != 0)
		{
			m_agents[id].active = 0;
			--m_nbActiveAgents;
		}
	}
}

unsigned dtCrowd::getActiveAgents(const dtCrowdAgent** agents, const unsigned maxAgents)
{
	int n = 0;
	for (unsigned i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active) continue;
		if (n < maxAgents)
			agents[n++] = &m_agents[i];
	}
	return n;
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

void dtCrowd::updateVelocity(const float dt, unsigned* agentsIdx, unsigned nbIdx)
{
	nbIdx = (nbIdx < m_maxAgents) ? nbIdx : m_maxAgents;
	
	// If we want to update every agent
	if (agentsIdx == 0)
	{
		agentsIdx = m_agentsToUpdate;
		nbIdx = m_maxAgents;
	}

	for (unsigned i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;
		
		if (ag->behavior)
			ag->behavior->update(*m_crowdQuery, *ag, *ag, dt);
	}

	// Fake dynamic constraint
	for (unsigned i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		const float maxDelta = ag->maxAcceleration * dt;
		float dv[3];
		dtVsub(dv, ag->desiredVelocity, ag->velocity);
		float ds = dtVlen(dv);

		if (ds > maxDelta)
			dtVscale(dv, dv, maxDelta/ds);

		dtVadd(ag->velocity, ag->velocity, dv);
	}
}

void dtCrowd::updatePosition(const float dt, unsigned* agentsIdx, unsigned nbIdx)
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

	for (unsigned i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;

		m_crowdQuery->getNavMeshQuery()->findNearestPoly(ag->position, m_crowdQuery->getQueryExtents(), m_crowdQuery->getQueryFilter(), currentPosPoly + i, currentPos + (i * 3));
	}

	// Integrate.
	for (unsigned i = 0; i < nbIdx; ++i)
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

	for (unsigned iter = 0; iter < 4; ++iter)
	{
		for (unsigned i = 0; i < nbIdx; ++i)
		{
			dtCrowdAgent* ag = 0;

			if (!getActiveAgent(&ag, agentsIdx[i]))
				continue;

			const int idx0 = getAgentIndex(ag);

			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			float* disp = m_disp[agentsIdx[i]];
			dtVset(disp, 0, 0, 0);

			float w = 0;

			for (unsigned j = 0; j < m_agentsEnv[ag->id].nbNeighbors; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[m_agentsEnv[ag->id].neighbors[j].idx];
				const int idx1 = getAgentIndex(nei);

				float diff[3];
				dtVsub(diff, ag->position, nei->position);
				diff[1] = 0;

				float dist = dtVlenSqr(diff);

				if (dist > dtSqr(ag->radius + nei->radius) + EPSILON)
					continue;

				dist = sqrtf(dist);
				float pen = (ag->radius + nei->radius) - dist;
				if (dist < EPSILON)
				{
					// Agents on top of each other, try to choose diverging separation directions.
					if (idx0 > idx1)
						dtVset(diff, -ag->desiredVelocity[2], 0, ag->desiredVelocity[0]);
					else
						dtVset(diff, ag->desiredVelocity[2], 0, -ag->desiredVelocity[0]);
					pen = 0.01f;
				}
				else
				{
					pen = (1.0f / dist) * (pen * 0.5f) * COLLISION_RESOLVE_FACTOR;
				}

				dtVmad(disp, disp, diff, pen);			

				w += 1.0f;
			}

			if (w > EPSILON)
			{
				const float iw = 1.0f / w;
				dtVscale(disp, disp, iw);
			}
		}
		
		for (unsigned i = 0; i < nbIdx; ++i)
		{
			dtCrowdAgent* ag = 0;

			if (!getActiveAgent(&ag, agentsIdx[i]))
				continue;

			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			float* disp = m_disp[agentsIdx[i]];

			dtVadd(ag->position, ag->position, disp);
		}
	}
	
	for (unsigned i = 0; i < nbIdx; ++i)
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
		m_crowdQuery->getNavMeshQuery()->moveAlongSurface(currentPosPoly[i], currentPos + (i * 3), ag->position, m_crowdQuery->getQueryFilter(), newPos, 
			visited, &visitedCount, dtPathCorridor::MAX_VISITED);

		// Get valid constrained position back.
		float newHeight = *(currentPos + (i * 3) + 1);
		m_crowdQuery->getNavMeshQuery()->getPolyHeight(currentPosPoly[i], newPos, &newHeight);
		newPos[1] = newHeight;

		dtVcopy(ag->position, newPos);
	}

	// Update agents using off-mesh connection.
	for (unsigned i = 0; i < nbIdx; ++i)
	{
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, agentsIdx[i]))
			continue;


		ag->offmeshElaspedTime += dt;
		if (ag->offmeshElaspedTime > ag->offmeshTotalTime)
		{
			// Prepare agent for walking.
			ag->state = DT_CROWDAGENT_STATE_WALKING;
			continue;
		}

		// Update position
		const float ta = ag->offmeshTotalTime * 0.15f;
		const float tb = ag->offmeshTotalTime;
		if (ag->offmeshElaspedTime < ta)
		{
			const float u = tween(ag->offmeshTotalTime, 0.0, ta);
			dtVlerp(ag->position, ag->offmeshInitPos, ag->offmeshStartPos, u);
		}
		else
		{
			const float u = tween(ag->offmeshTotalTime, ta, tb);
			dtVlerp(ag->position, ag->offmeshStartPos, ag->offmeshEndPos, u);
		}

		// Update velocity.
		dtVset(ag->velocity, 0,0,0);
		dtVset(ag->desiredVelocity, 0,0,0);
	}

	dtFree(currentPosPoly);
	dtFree(currentPos);
}

void dtCrowd::updateEnvironment(unsigned* agentsIdx, unsigned nbIdx)
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
		dtCrowdAgent* ag = 0;

		if (!getActiveAgent(&ag, i))
			continue;

		const float* p = ag->position;
		const float r = ag->radius;
		m_crowdQuery->getProximityGrid()->addItem((unsigned short)i, p[0]-r, p[2]-r, p[0]+r, p[2]+r);
	}

	// Get nearby navmesh segments and agents to collide with.
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
		const float updateThr = m_collisionQueryRange * 0.25f;
		if (dtVdist2DSqr(ag->position, m_agentsEnv[ag->id].boundary.getCenter()) > dtSqr(updateThr) ||
			!m_agentsEnv[ag->id].boundary.isValid(m_crowdQuery->getNavMeshQuery(), m_crowdQuery->getQueryFilter()))
		{
			dtPolyRef ref;
			float nearest[3];
			m_crowdQuery->getNavMeshQuery()->findNearestPoly(ag->position, m_crowdQuery->getQueryExtents(), m_crowdQuery->getQueryFilter(), &ref, nearest);

			m_agentsEnv[ag->id].boundary.update(ref, ag->position, m_collisionQueryRange, 
				m_crowdQuery->getNavMeshQuery(), m_crowdQuery->getQueryFilter());
		}
		// Query neighbour agents
		m_agentsEnv[ag->id].nbNeighbors = getNeighbours(ag->position, ag->height, m_collisionQueryRange,
			ag, m_agentsEnv[ag->id].neighbors, DT_CROWDAGENT_MAX_NEIGHBOURS,
			m_agents, m_nbActiveAgents, m_crowdQuery->getProximityGrid());

		for (unsigned j = 0; j < m_agentsEnv[ag->id].nbNeighbors; j++)
			m_agentsEnv[ag->id].neighbors[j].idx = getAgentIndex(&m_agents[m_agentsEnv[ag->id].neighbors[j].idx]);
	}
}
	
void dtCrowd::update(const float dt, unsigned* indexList, unsigned nbIndex)
{
	updateEnvironment(indexList, nbIndex);
	updateVelocity(dt, indexList, nbIndex);
	updatePosition(dt, indexList, nbIndex);
}

bool dtCrowd::updateAgentPosition(unsigned id, const float* position)
{
	if (id < m_maxAgents)
	{
		dtCrowdAgent& ag = m_agents[id];
		dtPolyRef ref = 0;
		float nearestPosition[] = {0, 0, 0};

		if (dtStatusFailed(m_crowdQuery->getNavMeshQuery()->findNearestPoly(position, m_crowdQuery->getQueryExtents(), m_crowdQuery->getQueryFilter(), &ref, nearestPosition)))
			return false;

		// If no polygons have been found, it's a failure
		if (ref == 0)
			return false;
		
		dtVset(ag.desiredVelocity, 0, 0, 0);
		dtVset(ag.velocity, 0, 0, 0);
		dtVcopy(ag.position, nearestPosition);

		ag.state = DT_CROWDAGENT_STATE_WALKING;
		
		return true;
	}

	return false;
}

bool dtCrowd::agentIsMoving(const dtCrowdAgent& ag) const
{
	if (ag.id >= m_maxAgents)
		return false;

	return (dtVlen(m_agents[ag.id].velocity) > EPSILON);
}

bool dtCrowd::applyAgent(const dtCrowdAgent& ag)
{
	if (ag.id >= m_maxAgents)
		return false;

	// Find nearest position on navmesh and place the agent there.
	float nearest[3];
	dtPolyRef ref;
	m_crowdQuery->getNavMeshQuery()->findNearestPoly(ag.position, m_crowdQuery->getQueryExtents(), 
		m_crowdQuery->getQueryFilter(), &ref, nearest);

	// If a position could not be found on the navigation mesh, then we do not apply the changes
	if (!ref)
		return false;

	m_agents[ag.id] = ag;

	// Checking out of bound limits
	m_agents[ag.id].radius = (m_agents[ag.id].radius < 0) ? 0 : m_agents[ag.id].radius;
	m_agents[ag.id].height = (m_agents[ag.id].height <= 0) ? 0.01f : m_agents[ag.id].height;
	m_agents[ag.id].maxAcceleration = (m_agents[ag.id].maxAcceleration < 0) ? 0 : m_agents[ag.id].maxAcceleration;
	m_agents[ag.id].maxSpeed = (m_agents[ag.id].maxSpeed < 0) ? 0 : m_agents[ag.id].maxSpeed;

	return true;
}

bool dtCrowd::setAgentBehavior(unsigned id, dtBehavior* behavior)
{
	if (id < m_maxAgents)
	{
		m_agents[id].behavior = behavior;
		return true;
	}

	return false;
}

unsigned dtCrowd::getAgents(const unsigned* ids, unsigned size, const dtCrowdAgent** agents) const
{
	return m_crowdQuery->getAgents(ids, size, agents);
}

float dtCrowd::getCollisionRange() const
{
	return m_collisionQueryRange;
}

dtCrowdQuery::~dtCrowdQuery()
{
	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFreeNavMeshQuery(m_navMeshQuery);
	m_navMeshQuery = 0;
}

dtCrowdQuery::dtCrowdQuery(unsigned maxAgents, const dtCrowdAgent* agents, const dtCrowdAgentEnvironment* env)
	: m_agents(agents),
	m_maxAgents(maxAgents),
	m_agentsEnv(env)
{
	m_navMeshQuery = dtAllocNavMeshQuery();
	m_grid = dtAllocProximityGrid();
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

const float* dtCrowdQuery::getQueryExtents() const
{ 
	return m_ext; 
}

const dtNavMeshQuery* dtCrowdQuery::getNavMeshQuery() const
{ 
	return m_navMeshQuery; 
}

const dtQueryFilter* dtCrowdQuery::getQueryFilter() const
{ 
	return &m_filter; 
}

const dtProximityGrid* dtCrowdQuery::getProximityGrid() const
{ 
	return m_grid; 
}

const dtCrowdAgent* dtCrowdQuery::getAgent(const unsigned id) const
{
	if (id < m_maxAgents)
		return &m_agents[id];

	return 0;
}

void dtCrowdQuery::fetchAgent(dtCrowdAgent& ag, unsigned id) const
{
	if (id < m_maxAgents)
		ag = m_agents[id];
}

unsigned dtCrowdQuery::getAgents(const unsigned* ids, unsigned size, const dtCrowdAgent** agents) const
{
	if (!ids || !agents)
		return 0;

	int nbFound = 0;

	for (unsigned i = 0; i < size; ++i)
	{
		unsigned id = ids[i];

		if (id >= m_maxAgents)
			continue;

		++nbFound;
		agents[i] = &m_agents[id];
	}

	return nbFound;
}

const dtCrowdAgentEnvironment* dtCrowdQuery::getAgentEnvironment(unsigned id) const
{
	if (id < m_maxAgents)
		return &m_agentsEnv[id];

	return 0;
}

dtCrowdAgentEnvironment::dtCrowdAgentEnvironment() 
	: nbNeighbors(0) 
{
	boundary.reset();
}

dtCrowdAgentEnvironment::~dtCrowdAgentEnvironment()
{
}
