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

#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"

#include <new>

#include <cmath>
#include <cstring>


dtPathFollowing::dtPathFollowing(unsigned nbMaxAgents) :
	dtParametrizedBehavior<dtPathFollowingParams>(nbMaxAgents),
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

bool dtPathFollowing::init(dtCrowdQuery& crowdQuery, dtNavMesh* navMesh, dtCrowdAgent* agents, int maxAgents)
{
	purge();

	m_navMeshQuery = crowdQuery.getNavMeshQuery();
	m_pathResult = (dtPolyRef*) dtAlloc(sizeof(dtPolyRef) * MAX_PATH_RES, DT_ALLOC_PERM);

	m_agents = agents;
	m_maxAgents = maxAgents;
	m_maxPathRes = MAX_PATH_RES;
	m_agentAnims = crowdQuery.getAgentsAnims();
	m_filter = crowdQuery.getQueryFilter();

	if (!m_pathQueue.init(m_maxPathRes, m_maxPathQueueNodes, navMesh))
		return false;

	if (!m_pathResult)
		return false;

	if (!m_navMeshQuery || !m_filter)
		return false;
	
	dtVcopy(m_ext, crowdQuery.getQueryExtents());

	return true;
}

void dtPathFollowing::purge()
{
}

void dtPathFollowing::prepare(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, const float dt, dtPathFollowingParams* agParams)
{
	checkPathValidity(oldAgent, newAgent, dt, agParams);
	updateMoveRequest();
	updateTopologyOptimization(oldAgent, dt, agParams);
}

void dtPathFollowing::getVelocity(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, dtPathFollowingParams* agParams)
{
	if (oldAgent->state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE)
		return;

	float dvel[3] = {0,0,0};

	if (agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
	{
		dtVcopy(dvel, agParams->targetPos);
		newAgent->desiredSpeed = dtVlen(agParams->targetPos);
	}
	else
	{
		// Calculate steering direction.
		if (oldAgent->updateFlags & DT_CROWD_ANTICIPATE_TURNS)
			calcSmoothSteerDirection(oldAgent, dvel, agParams);
		else
			calcStraightSteerDirection(oldAgent, dvel, agParams);

		// Calculate speed scale, which tells the agent to slowdown at the end of the path.
		const float slowDownRadius = oldAgent->radius * 2;	// TODO: make less hacky.
		const float speedScale = getDistanceToGoal(oldAgent, slowDownRadius, agParams) / slowDownRadius;

		newAgent->desiredSpeed = oldAgent->maxSpeed;
		dtVscale(dvel, dvel, newAgent->desiredSpeed * speedScale);
	}
		
	dtVcopy(newAgent->dvel, dvel);
}


float dtPathFollowing::getDistanceToGoal(const dtCrowdAgent* ag, const float range, dtPathFollowingParams* agParams)
{
	if (!agParams->ncorners)
		return range;

	const bool endOfPath = (agParams->cornerFlags[agParams->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	if (endOfPath)
		return dtMin(dtVdist2D(ag->npos, &agParams->cornerVerts[(agParams->ncorners-1)*3]), range);

	return range;
}

void dtPathFollowing::calcSmoothSteerDirection(const dtCrowdAgent* ag, float* dir, dtPathFollowingParams* agParams)
{
	if (!agParams->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}

	const int ip0 = 0;
	const int ip1 = dtMin(1, agParams->ncorners-1);
	const float* p0 = &agParams->cornerVerts[ip0*3];
	const float* p1 = &agParams->cornerVerts[ip1*3];

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

void dtPathFollowing::calcStraightSteerDirection(const dtCrowdAgent* ag, float* dir, dtPathFollowingParams* agParams)
{
	if (!agParams->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &agParams->cornerVerts[0], ag->npos);
	dir[1] = 0;
	dtVnormalize(dir);
}

void dtPathFollowing::triggerOffMeshConnections(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, dtPathFollowingParams* agParams)
{
	// Trigger off-mesh connections (depends on corners).

	if (oldAgent->state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE || agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;

	// Check 
	const float triggerRadius = oldAgent->radius * 2.25f;
	if (overOffmeshConnection(oldAgent, triggerRadius, agParams))
	{
		// Prepare to off-mesh connection.
		const int idx = oldAgent - m_agents;
		dtCrowdAgentAnimation* anim = &m_agentAnims[idx];

		// Adjust the path over the off-mesh connection.
		dtPolyRef refs[2];
		if (agParams->corridor.moveOverOffmeshConnection(agParams->cornerPolys[agParams->ncorners-1], refs,
			anim->startPos, anim->endPos, m_navMeshQuery))
		{
			dtVcopy(anim->initPos, oldAgent->npos);
			anim->polyRef = refs[1];
			anim->active = 1;
			anim->t = 0.0f;
			anim->tmax = (dtVdist2D(anim->startPos, anim->endPos) / oldAgent->maxSpeed) * 0.5f;

			newAgent->state = DT_CROWDAGENT_STATE_OFFMESH;
			agParams->ncorners = 0;
			return;
		}
		else
		{
			// Path validity check will ensure that bad/blocked connections will be replanned.
		}
	}
}

void dtPathFollowing::getNextCorner(const dtCrowdAgent* ag, dtPathFollowingParams* agParams)
{
	dtCrowdAgentDebugInfo* debug = getBehaviorParams(*ag)->debugInfos;
	const int debugIdx = debug ? debug->idx : -1;

	if (ag->state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE || agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;


	// Find corners for steering
	agParams->ncorners = agParams->corridor.findCorners(agParams->cornerVerts, agParams->cornerFlags, agParams->cornerPolys,
		dtPathFollowingParams::DT_CROWDAGENT_MAX_CORNERS, m_navMeshQuery, m_filter);

	// Check to see if the corner after the next corner is directly visible,
	// and short cut to there.
	if ((ag->updateFlags & DT_CROWD_OPTIMIZE_VIS) && agParams->ncorners > 0)
	{
		dtPathFollowingParams* params = this->getBehaviorParams(*ag);
		float pathOptRange = (params) ? params->pathOptimizationRange : 0.f;

		const float* target = &agParams->cornerVerts[dtMin(1,agParams->ncorners-1)*3];
		agParams->corridor.optimizePathVisibility(target, pathOptRange, m_navMeshQuery, m_filter);

		// Copy data for debug purposes.
		if (debugIdx == this->getBehaviorParams(*ag)->debugIndex)
		{
			dtVcopy(debug->optStart, agParams->corridor.getPos());
			dtVcopy(debug->optEnd, target);
		}
	}
	else
	{
		// Copy data for debug purposes.
		if (debugIdx == getBehaviorParams(*ag)->debugIndex)
		{
			dtVset(debug->optStart, 0,0,0);
			dtVset(debug->optEnd, 0,0,0);
		}
	}
}

void dtPathFollowing::updateTopologyOptimization(const dtCrowdAgent* ag, const float dt, dtPathFollowingParams* agParams)
{
	const float OPT_TIME_THR = 0.5f; // seconds
	const int OPT_MAX_AGENTS = 1;
	dtCrowdAgent* queue[OPT_MAX_AGENTS];
	int nqueue = 0;

	if (ag->state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE || agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;
	if ((ag->updateFlags & DT_CROWD_OPTIMIZE_TOPO) == 0)
		return;
	agParams->topologyOptTime += dt;

	if (agParams->topologyOptTime >= OPT_TIME_THR)
		nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		agParams->corridor.optimizePathTopology(m_navMeshQuery, m_filter);
		agParams->topologyOptTime = 0;
	}
}

int dtPathFollowing::addToOptQueue(const dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	dtPathFollowingParams* newAgParams = getBehaviorParams(*newag);
	if (!newAgParams)
		return 0;

	dtPathFollowingParams* agParams = getBehaviorParams(*agents[nagents-1]);
	if (!agParams)
		return 0;

	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (newAgParams->topologyOptTime <= agParams->topologyOptTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
		{
			dtPathFollowingParams* pfParams = getBehaviorParams(*agents[i]);
			if (!pfParams)
				continue;

			if (newAgParams->topologyOptTime >= pfParams->topologyOptTime)
				break;
		}

		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);

		dtAssert(tgt+n <= maxAgents);

		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*)*n);
		slot = i;
	}

	agents[slot] = const_cast<dtCrowdAgent*>(newag);

	return dtMin(nagents+1, maxAgents);
}

void dtPathFollowing::checkPathValidity(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, const float dt, dtPathFollowingParams* agParams)
{
	static const int CHECK_LOOKAHEAD = 10;
	static const float TARGET_REPLAN_DELAY = 1.0; // seconds
	
	if (oldAgent->state != DT_CROWDAGENT_STATE_WALKING)
		return;

	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE || agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;


	agParams->targetReplanTime += dt;

	bool replan = false;
	// First check that the current location is valid.
	const int idx = oldAgent - m_agents;
	float agentPos[3];
	dtPolyRef agentRef = agParams->corridor.getFirstPoly();
	dtVcopy(agentPos, oldAgent->npos);
	if (!m_navMeshQuery->isValidPolyRef(agentRef, m_filter))
	{
		// Current location is not valid, try to reposition.
		// TODO: this can snap agents, how to handle that?
		float nearest[3];
		agentRef = 0;
		m_navMeshQuery->findNearestPoly(oldAgent->npos, m_ext, m_filter, &agentRef, nearest);
		dtVcopy(agentPos, nearest);

		if (!agentRef)
		{
			// Could not find location in navmesh, set state to invalid.
			agParams->corridor.reset(0, agentPos);
			newAgent->state = DT_CROWDAGENT_STATE_INVALID;
			return;
		}

		// Make sure the first polygon is valid, but leave other valid
		// polygons in the path so that replanner can adjust the path better.
		agParams->corridor.fixPathStart(agentRef, agentPos);
		//			agParams->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
		dtVcopy(newAgent->npos, agentPos);

		replan = true;
	}
		
	// Try to recover move request position.
	if (agParams->targetState != DT_CROWDAGENT_TARGET_NONE && agParams->targetState != DT_CROWDAGENT_TARGET_FAILED)
	{
		if (!m_navMeshQuery->isValidPolyRef(agParams->targetRef, m_filter))
		{
			// Current target is not valid, try to reposition.
			float nearest[3];
			m_navMeshQuery->findNearestPoly(agParams->targetPos, m_ext, m_filter, &agParams->targetRef, nearest);
			dtVcopy(agParams->targetPos, nearest);
			replan = true;
		}
		if (!agParams->targetRef)
		{
			// Failed to reposition target, fail moverequest.
			agParams->corridor.reset(agentRef, agentPos);
			agParams->targetState = DT_CROWDAGENT_TARGET_NONE;
		}
	}

	// If nearby corridor is not valid, replan.
	if (!agParams->corridor.isValid(CHECK_LOOKAHEAD, m_navMeshQuery, m_filter))
	{
		// Fix current path.
		// agParams->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
		// ag->boundary.reset();
		replan = true;
	}

	// If the end of the path is near and it is not the requested location, replan.
	if (agParams->targetState == DT_CROWDAGENT_TARGET_VALID)
	{
		if (agParams->targetReplanTime > TARGET_REPLAN_DELAY &&
			agParams->corridor.getPathCount() < CHECK_LOOKAHEAD &&
			agParams->corridor.getLastPoly() != agParams->targetRef)
			replan = true;
	}

	// Try to replan path to goal.
	if (replan)
	{
		if (agParams->targetState != DT_CROWDAGENT_TARGET_NONE)
		{
			requestMoveTargetReplan(idx, agParams->targetRef, agParams->targetPos);
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
bool dtPathFollowing::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	if (!ref)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	dtPathFollowingParams* agParams = getBehaviorParams(*ag);
	if (!agParams)
		return false;

	// Initialize request.
	agParams->targetRef = ref;
	dtVcopy(agParams->targetPos, pos);
	agParams->targetPathqRef = DT_PATHQ_INVALID;
	agParams->targetReplan = false;
	if (agParams->targetRef)
		agParams->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		agParams->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

bool dtPathFollowing::requestMoveVelocity(const int idx, const float* vel)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	dtPathFollowingParams* agParams = getBehaviorParams(*ag);
	if (!agParams)
		return false;

	// Initialize request.
	agParams->targetRef = 0;
	dtVcopy(agParams->targetPos, vel);
	agParams->targetPathqRef = DT_PATHQ_INVALID;
	agParams->targetReplan = false;
	agParams->targetState = DT_CROWDAGENT_TARGET_VELOCITY;

	return true;
}

bool dtPathFollowing::resetMoveTarget(const int idx)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	dtPathFollowingParams* agParams = getBehaviorParams(*ag);
	if (!agParams)
		return false;

	// Initialize request.
	agParams->targetRef = 0;
	dtVset(agParams->targetPos, 0,0,0);
	agParams->targetPathqRef = DT_PATHQ_INVALID;
	agParams->targetReplan = false;
	agParams->targetState = DT_CROWDAGENT_TARGET_NONE;

	return true;
}

bool dtPathFollowing::requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	dtPathFollowingParams* agParams = getBehaviorParams(*ag);
	if (!agParams)
		return false;

	// Initialize request.
	agParams->targetRef = ref;
	dtVcopy(agParams->targetPos, pos);
	agParams->targetPathqRef = DT_PATHQ_INVALID;
	agParams->targetReplan = true;
	if (agParams->targetRef)
		agParams->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		agParams->targetState = DT_CROWDAGENT_TARGET_FAILED;

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

		dtPathFollowingParams* pfParams = getBehaviorParams(*ag);
		if (!pfParams)
			continue;

		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			continue;
		if (pfParams->targetState == DT_CROWDAGENT_TARGET_NONE || pfParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (pfParams->targetState == DT_CROWDAGENT_TARGET_REQUESTING)
		{
			const dtPolyRef* path = pfParams->corridor.getPath();
			const int npath = pfParams->corridor.getPathCount();
			dtAssert(npath);

			static const int MAX_RES = 32;
			float reqPos[3];
			dtPolyRef reqPath[MAX_RES];	// The path to the request location
			int reqPathCount = 0;

			// Quick seach towards the goal.
			static const int MAX_ITER = 20;
			m_navMeshQuery->initSlicedFindPath(path[0], pfParams->targetRef, ag->npos, pfParams->targetPos, m_filter);
			m_navMeshQuery->updateSlicedFindPath(MAX_ITER, 0);
			dtStatus status = 0;
			if (pfParams->targetReplan) // && npath > 10)
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
				if (reqPath[reqPathCount-1] != pfParams->targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					status = m_navMeshQuery->closestPointOnPoly(reqPath[reqPathCount-1], pfParams->targetPos, reqPos);
					if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					dtVcopy(reqPos, pfParams->targetPos);
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

			pfParams->corridor.setCorridor(reqPos, reqPath, reqPathCount);

			if (reqPath[reqPathCount-1] == pfParams->targetRef)
			{
				pfParams->targetState = DT_CROWDAGENT_TARGET_VALID;
				pfParams->targetReplanTime = 0.0;
			}
			else
			{
				// The path is longer or potentially unreachable, full plan.
				pfParams->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
			}
		}

		if (pfParams->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			nqueue = addToPathQueue(ag, queue, nqueue, PATH_MAX_AGENTS);
		}
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];

		dtPathFollowingParams* pfParams = getBehaviorParams(*ag);
		if (!pfParams)
			continue;

		pfParams->targetPathqRef = m_pathQueue.request(pfParams->corridor.getLastPoly(), pfParams->targetRef,
			pfParams->corridor.getTarget(), pfParams->targetPos, m_filter);
		if (pfParams->targetPathqRef != DT_PATHQ_INVALID)
			pfParams->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
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

		dtPathFollowingParams* pfParams = getBehaviorParams(*ag);
		if (!pfParams)
			continue;

		if (pfParams->targetState == DT_CROWDAGENT_TARGET_NONE || pfParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (pfParams->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			status = m_pathQueue.getRequestStatus(pfParams->targetPathqRef);
			if (dtStatusFailed(status))
			{
				// Path find failed, retry if the target location is still valid.
				pfParams->targetPathqRef = DT_PATHQ_INVALID;

				if (pfParams->targetRef)
					pfParams->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
				else
					pfParams->targetState = DT_CROWDAGENT_TARGET_FAILED;

				pfParams->targetReplanTime = 0.0;
			}
			else if (dtStatusSucceed(status))
			{
				const dtPolyRef* path = pfParams->corridor.getPath();
				const int npath = pfParams->corridor.getPathCount();
				dtAssert(npath);

				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, pfParams->targetPos);

				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = 0;
				status = m_pathQueue.getPathResult(pfParams->targetPathqRef, res, &nres, m_maxPathRes);
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
					if (res[nres-1] != pfParams->targetRef)
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
					pfParams->corridor.setCorridor(targetPos, res, nres);

					pfParams->targetState = DT_CROWDAGENT_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
					pfParams->targetState = DT_CROWDAGENT_TARGET_FAILED;
				}

				pfParams->targetReplanTime = 0.0;
			}
		}
	}

}

int dtPathFollowing::addToPathQueue(const dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	dtPathFollowingParams* pfParams = getBehaviorParams(*newag);
	if (!pfParams)
		return 0;

	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (pfParams->targetReplanTime <= getBehaviorParams(*agents[nagents-1])->targetReplanTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (pfParams->targetReplanTime >= getBehaviorParams(*agents[i])->targetReplanTime)
				break;

		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);

		dtAssert(tgt+n <= maxAgents);

		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*)*n);
		slot = i;
	}

	agents[slot] = const_cast<dtCrowdAgent*>(newag);

	return dtMin(nagents+1, maxAgents);
}

bool dtPathFollowing::overOffmeshConnection(const dtCrowdAgent* ag, const float radius, dtPathFollowingParams* agParams)
{
	if (!agParams->ncorners)
		return false;

	const bool offMeshConnection = (agParams->cornerFlags[agParams->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (offMeshConnection)
	{
		const float distSq = dtVdist2DSqr(ag->npos, &agParams->cornerVerts[(agParams->ncorners-1)*3]);
		if (distSq < radius*radius)
			return true;
	}

	return false;
}

void dtPathFollowing::update(const dtCrowdAgent* oldAgent, dtCrowdAgent* newAgent, float dt)
{
	dtPathFollowingParams* agParams = getBehaviorParams(*oldAgent);
	if (!agParams)
		return;

	prepare(oldAgent, newAgent, dt, agParams);
	getNextCorner(oldAgent, agParams);
	triggerOffMeshConnections(oldAgent, newAgent, agParams);
	getVelocity(oldAgent, newAgent, agParams);
}

dtPathFollowing* dtPathFollowing::allocate(unsigned nbMaxAgents)
{
	void* mem = dtAlloc(sizeof(dtPathFollowing), DT_ALLOC_PERM);

	if (mem)
		return new(mem) dtPathFollowing(nbMaxAgents);

	return 0;
}

void dtPathFollowing::free(dtPathFollowing* ptr)
{
	if (!ptr)
		return;

	ptr->~dtPathFollowing();
	dtFree(ptr);
	ptr = 0;
}

dtPathFollowingParams::dtPathFollowingParams() 
	: debugInfos(0)
	, debugIndex(0)
	, pathOptimizationRange(6.f)
	, ncorners(0)
	, topologyOptTime(0)
{
	corridor.init(MAX_PATH_RES);
}
