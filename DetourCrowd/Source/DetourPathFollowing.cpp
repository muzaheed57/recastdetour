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

#include "DetourPathFollowing.h"

#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"

#include <new>

#include <cmath>
#include <cstring>


dtPathFollowing::dtPathFollowing() :
	dtSteeringBehavior(),
	m_navMeshQuery(0),
	m_agents(0),
	m_pathResult(0),
	m_maxAgents(0),
	m_maxPathRes(0),
	m_maxCommonNodes(512),
	m_maxPathQueueNodes(4096),
	m_maxIterPerUpdate(100)
{

}

dtPathFollowing::~dtPathFollowing()
{
	purge();
}

bool dtPathFollowing::init(dtNavMesh* navMesh, float* ext, int maxPathRes, dtCrowdAgent* agents, int maxAgents, dtCrowdAgentAnimation* anims)
{
	purge();

	m_navMeshQuery = dtAllocNavMeshQuery();
	m_pathResult = (dtPolyRef*) dtAlloc(sizeof(dtPolyRef) * maxPathRes, DT_ALLOC_PERM);

	m_agents = agents;
	m_maxAgents = maxAgents;
	m_maxPathRes = maxPathRes;
	m_agentAnims = anims;

	if (!m_navMeshQuery || !m_pathResult)
		return false;

	if (dtStatusFailed(m_navMeshQuery->init(navMesh, m_maxCommonNodes)))
		return false;

	if (!m_pathQueue.init(maxPathRes, m_maxPathQueueNodes, navMesh))
		return false;

	dtVset(m_ext, ext[0], ext[1], ext[2]);

	return true;
}

void dtPathFollowing::purge()
{
	dtFreeNavMeshQuery(m_navMeshQuery);
	m_navMeshQuery = 0;
}

void dtPathFollowing::prepare(dtCrowdAgent* ag, const float dt)
{
	checkPathValidity(ag, dt);
	updateMoveRequest();
	updateTopologyOptimization(ag, dt);
}

void dtPathFollowing::getVelocity(dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent)
{
	if (oldAgent->state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (oldAgent->targetState == DT_CROWDAGENT_TARGET_NONE)
		return;

	float dvel[3] = {0,0,0};

	if (oldAgent->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
	{
		dtVcopy(dvel, oldAgent->targetPos);
		newAgent->desiredSpeed = dtVlen(oldAgent->targetPos);
	}
	else
	{
		// Calculate steering direction.
		if (oldAgent->params.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
			calcSmoothSteerDirection(oldAgent, dvel);
		else
			calcStraightSteerDirection(oldAgent, dvel);

		// Calculate speed scale, which tells the agent to slowdown at the end of the path.
		const float slowDownRadius = oldAgent->params.radius * 2;	// TODO: make less hacky.
		const float speedScale = getDistanceToGoal(oldAgent, slowDownRadius) / slowDownRadius;

		newAgent->desiredSpeed = oldAgent->params.maxSpeed;
		dtVscale(dvel, dvel, newAgent->desiredSpeed * speedScale);
	}
		
	dtVcopy(newAgent->dvel, dvel);
}

void dtPathFollowing::computeForce(const dtCrowdAgent* ag, float* force)
{

}


float dtPathFollowing::getDistanceToGoal(const dtCrowdAgent* ag, const float range)
{
	if (!ag->ncorners)
		return range;

	const bool endOfPath = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	if (endOfPath)
		return dtMin(dtVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]), range);

	return range;
}

void dtPathFollowing::calcSmoothSteerDirection(const dtCrowdAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}

	const int ip0 = 0;
	const int ip1 = dtMin(1, ag->ncorners-1);
	const float* p0 = &ag->cornerVerts[ip0*3];
	const float* p1 = &ag->cornerVerts[ip1*3];

	float dir0[3], dir1[3];
	dtVsub(dir0, p0, ag->npos);
	dtVsub(dir1, p1, ag->npos);
	dir0[1] = 0;
	dir1[1] = 0;

	float len0 = dtVlen(dir0);
	float len1 = dtVlen(dir1);
	if (len1 > 0.001f)
		dtVscale(dir1,dir1,1.0f/len1);

	dir[0] = dir0[0] - dir1[0]*len0*0.5f;
	dir[1] = 0;
	dir[2] = dir0[2] - dir1[2]*len0*0.5f;

	dtVnormalize(dir);
}

void dtPathFollowing::calcStraightSteerDirection(const dtCrowdAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &ag->cornerVerts[0], ag->npos);
	dir[1] = 0;
	dtVnormalize(dir);
}

void dtPathFollowing::triggerOffMeshConnections(dtCrowdAgent* ag)
{
	// Trigger off-mesh connections (depends on corners).

	if (ag->state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;

	// Check 
	const float triggerRadius = ag->params.radius*2.25f;
	if (overOffmeshConnection(ag, triggerRadius))
	{
		// Prepare to off-mesh connection.
		const int idx = ag - m_agents;
		dtCrowdAgentAnimation* anim = &m_agentAnims[idx];

		// Adjust the path over the off-mesh connection.
		dtPolyRef refs[2];
		if (ag->corridor.moveOverOffmeshConnection(ag->cornerPolys[ag->ncorners-1], refs,
			anim->startPos, anim->endPos, m_navMeshQuery))
		{
			dtVcopy(anim->initPos, ag->npos);
			anim->polyRef = refs[1];
			anim->active = 1;
			anim->t = 0.0f;
			anim->tmax = (dtVdist2D(anim->startPos, anim->endPos) / ag->params.maxSpeed) * 0.5f;

			ag->state = DT_CROWDAGENT_STATE_OFFMESH;
			ag->ncorners = 0;
			ag->nneis = 0;
			return;
		}
		else
		{
			// Path validity check will ensure that bad/blocked connections will be replanned.
		}
	}
}

void dtPathFollowing::getNextCorner(dtCrowdAgent* ag)
{
	dtCrowdAgentDebugInfo* debug = ag->params.pfDebug;
	const int debugIdx = debug ? debug->idx : -1;

	if (ag->state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;


	// Find corners for steering
	ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
		DT_CROWDAGENT_MAX_CORNERS, m_navMeshQuery, &m_filter);

	// Check to see if the corner after the next corner is directly visible,
	// and short cut to there.
	if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
	{
		const float* target = &ag->cornerVerts[dtMin(1,ag->ncorners-1)*3];
		ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navMeshQuery, &m_filter);

		// Copy data for debug purposes.
		if (debugIdx == ag->params.pfDebugIbdex)
		{
			dtVcopy(debug->optStart, ag->corridor.getPos());
			dtVcopy(debug->optEnd, target);
		}
	}
	else
	{
		// Copy data for debug purposes.
		if (debugIdx == ag->params.pfDebugIbdex)
		{
			dtVset(debug->optStart, 0,0,0);
			dtVset(debug->optEnd, 0,0,0);
		}
	}
}

void dtPathFollowing::updateTopologyOptimization(dtCrowdAgent* ag, const float dt)
{
	const float OPT_TIME_THR = 0.5f; // seconds
	const int OPT_MAX_AGENTS = 1;
	dtCrowdAgent* queue[OPT_MAX_AGENTS];
	int nqueue = 0;

	if (ag->state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;
	if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_TOPO) == 0)
		return;
	ag->topologyOptTime += dt;

	if (ag->topologyOptTime >= OPT_TIME_THR)
		nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->corridor.optimizePathTopology(m_navMeshQuery, &m_filter);
		ag->topologyOptTime = 0;
	}

}

int dtPathFollowing::addToOptQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (newag->topologyOptTime <= agents[nagents-1]->topologyOptTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->topologyOptTime >= agents[i]->topologyOptTime)
				break;

		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);

		dtAssert(tgt+n <= maxAgents);

		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*)*n);
		slot = i;
	}

	agents[slot] = newag;

	return dtMin(nagents+1, maxAgents);
}

void dtPathFollowing::checkPathValidity(dtCrowdAgent* ag, const float dt)
{
	static const int CHECK_LOOKAHEAD = 10;
	static const float TARGET_REPLAN_DELAY = 1.0; // seconds
	
	if (ag->state != DT_CROWDAGENT_STATE_WALKING)
		return;

	if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;


	ag->targetReplanTime += dt;

	bool replan = false;
	// First check that the current location is valid.
	const int idx = ag - m_agents;
	float agentPos[3];
	dtPolyRef agentRef = ag->corridor.getFirstPoly();
	dtVcopy(agentPos, ag->npos);
	if (!m_navMeshQuery->isValidPolyRef(agentRef, &m_filter))
	{
		// Current location is not valid, try to reposition.
		// TODO: this can snap agents, how to handle that?
		float nearest[3];
		agentRef = 0;
		m_navMeshQuery->findNearestPoly(ag->npos, m_ext, &m_filter, &agentRef, nearest);
		dtVcopy(agentPos, nearest);

		if (!agentRef)
		{
			// Could not find location in navmesh, set state to invalid.
			ag->corridor.reset(0, agentPos);
			ag->boundary.reset();
			ag->state = DT_CROWDAGENT_STATE_INVALID;
			return;
		}

		// Make sure the first polygon is valid, but leave other valid
		// polygons in the path so that replanner can adjust the path better.
		ag->corridor.fixPathStart(agentRef, agentPos);
		//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
		ag->boundary.reset();
		dtVcopy(ag->npos, agentPos);

		replan = true;
	}
		
	// Try to recover move request position.
	if (ag->targetState != DT_CROWDAGENT_TARGET_NONE && ag->targetState != DT_CROWDAGENT_TARGET_FAILED)
	{
		if (!m_navMeshQuery->isValidPolyRef(ag->targetRef, &m_filter))
		{
			// Current target is not valid, try to reposition.
			float nearest[3];
			m_navMeshQuery->findNearestPoly(ag->targetPos, m_ext, &m_filter, &ag->targetRef, nearest);
			dtVcopy(ag->targetPos, nearest);
			replan = true;
		}
		if (!ag->targetRef)
		{
			// Failed to reposition target, fail moverequest.
			ag->corridor.reset(agentRef, agentPos);
			ag->targetState = DT_CROWDAGENT_TARGET_NONE;
		}
	}

	// If nearby corridor is not valid, replan.
	if (!ag->corridor.isValid(CHECK_LOOKAHEAD, m_navMeshQuery, &m_filter))
	{
		// Fix current path.
		// ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
		// ag->boundary.reset();
		replan = true;
	}

	// If the end of the path is near and it is not the requested location, replan.
	if (ag->targetState == DT_CROWDAGENT_TARGET_VALID)
	{
		if (ag->targetReplanTime > TARGET_REPLAN_DELAY &&
			ag->corridor.getPathCount() < CHECK_LOOKAHEAD &&
			ag->corridor.getLastPoly() != ag->targetRef)
			replan = true;
	}

	// Try to replan path to goal.
	if (replan)
	{
		if (ag->targetState != DT_CROWDAGENT_TARGET_NONE)
		{
			requestMoveTargetReplan(idx, ag->targetRef, ag->targetPos);
		}
	}
}

bool dtPathFollowing::requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = true;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

void dtPathFollowing::updateMoveRequest()
{
	const int PATH_MAX_AGENTS = 8;
	dtCrowdAgent* queue[PATH_MAX_AGENTS];
	int nqueue = 0;

	// Fire off new requests.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];
		if (!ag->active)
			continue;
		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING)
		{
			const dtPolyRef* path = ag->corridor.getPath();
			const int npath = ag->corridor.getPathCount();
			dtAssert(npath);

			static const int MAX_RES = 32;
			float reqPos[3];
			dtPolyRef reqPath[MAX_RES];	// The path to the request location
			int reqPathCount = 0;

			// Quick seach towards the goal.
			static const int MAX_ITER = 20;
			m_navMeshQuery->initSlicedFindPath(path[0], ag->targetRef, ag->npos, ag->targetPos, &m_filter);
			m_navMeshQuery->updateSlicedFindPath(MAX_ITER, 0);
			dtStatus status = 0;
			if (ag->targetReplan) // && npath > 10)
			{
				// Try to use existing steady path during replan if possible.
				status = m_navMeshQuery->finalizeSlicedFindPathPartial(path, npath, reqPath, &reqPathCount, MAX_RES);
			}
			else
			{
				// Try to move towards target when goal changes.
				status = m_navMeshQuery->finalizeSlicedFindPath(reqPath, &reqPathCount, MAX_RES);
			}

			if (!dtStatusFailed(status) && reqPathCount > 0)
			{
				// In progress or succeed.
				if (reqPath[reqPathCount-1] != ag->targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					status = m_navMeshQuery->closestPointOnPoly(reqPath[reqPathCount-1], ag->targetPos, reqPos);
					if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					dtVcopy(reqPos, ag->targetPos);
				}
			}
			else
			{
				reqPathCount = 0;
			}

			if (!reqPathCount)
			{
				// Could not find path, start the request from current location.
				dtVcopy(reqPos, ag->npos);
				reqPath[0] = path[0];
				reqPathCount = 1;
			}

			ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
			ag->boundary.reset();

			if (reqPath[reqPathCount-1] == ag->targetRef)
			{
				ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				ag->targetReplanTime = 0.0;
			}
			else
			{
				// The path is longer or potentially unreachable, full plan.
				ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
			}
		}

		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			nqueue = addToPathQueue(ag, queue, nqueue, PATH_MAX_AGENTS);
		}
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->targetPathqRef = m_pathQueue.request(ag->corridor.getLastPoly(), ag->targetRef,
			ag->corridor.getTarget(), ag->targetPos, &m_filter);
		if (ag->targetPathqRef != DT_PATHQ_INVALID)
			ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
	}


	// Update requests.
	m_pathQueue.update(m_maxIterPerUpdate);

	dtStatus status;

	// Process path results.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];
		if (!ag->active)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			status = m_pathQueue.getRequestStatus(ag->targetPathqRef);
			if (dtStatusFailed(status))
			{
				// Path find failed, retry if the target location is still valid.
				ag->targetPathqRef = DT_PATHQ_INVALID;
				if (ag->targetRef)
					ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
				else
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				ag->targetReplanTime = 0.0;
			}
			else if (dtStatusSucceed(status))
			{
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();
				dtAssert(npath);

				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, ag->targetPos);

				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = 0;
				status = m_pathQueue.getPathResult(ag->targetPathqRef, res, &nres, m_maxPathRes);
				if (dtStatusFailed(status) || !nres)
					valid = false;

				// Merge result and existing path.
				// The agent might have moved whilst the request is
				// being processed, so the path may have changed.
				// We assume that the end of the path is at the same location
				// where the request was issued.

				// The last ref in the old path should be the same as
				// the location where the request was issued..
				if (valid && path[npath-1] != res[0])
					valid = false;

				if (valid)
				{
					// Put the old path infront of the old path.
					if (npath > 1)
					{
						// Make space for the old path.
						if ((npath-1)+nres > m_maxPathRes)
							nres = m_maxPathRes - (npath-1);

						memmove(res+npath-1, res, sizeof(dtPolyRef)*nres);
						// Copy old path in the beginning.
						memcpy(res, path, sizeof(dtPolyRef)*(npath-1));
						nres += npath-1;

						// Remove trackbacks
						for (int j = 0; j < nres; ++j)
						{
							if (j-1 >= 0 && j+1 < nres)
							{
								if (res[j-1] == res[j+1])
								{
									memmove(res+(j-1), res+(j+1), sizeof(dtPolyRef)*(nres-(j+1)));
									nres -= 2;
									j -= 2;
								}
							}
						}

					}

					// Check for partial path.
					if (res[nres-1] != ag->targetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						float nearest[3];
						status = m_navMeshQuery->closestPointOnPoly(res[nres-1], targetPos, nearest);
						if (dtStatusSucceed(status))
							dtVcopy(targetPos, nearest);
						else
							valid = false;
					}
				}

				if (valid)
				{
					// Set current corridor.
					ag->corridor.setCorridor(targetPos, res, nres);
					// Force to update boundary.
					ag->boundary.reset();
					ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				}

				ag->targetReplanTime = 0.0;
			}
		}
	}

}

int dtPathFollowing::addToPathQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (newag->targetReplanTime <= agents[nagents-1]->targetReplanTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->targetReplanTime >= agents[i]->targetReplanTime)
				break;

		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);

		dtAssert(tgt+n <= maxAgents);

		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*)*n);
		slot = i;
	}

	agents[slot] = newag;

	return dtMin(nagents+1, maxAgents);
}

bool dtPathFollowing::overOffmeshConnection(const dtCrowdAgent* ag, const float radius)
{
	if (!ag->ncorners)
		return false;

	const bool offMeshConnection = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (offMeshConnection)
	{
		const float distSq = dtVdist2DSqr(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]);
		if (distSq < radius*radius)
			return true;
	}

	return false;
}

void dtPathFollowing::update(dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	prepare(oldAgent, dt);
	getNextCorner(oldAgent);
	triggerOffMeshConnections(oldAgent);
	getVelocity(oldAgent, newAgent);
}
