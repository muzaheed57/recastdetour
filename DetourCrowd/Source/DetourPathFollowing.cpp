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

bool dtPathFollowing::init(dtCrowdQuery& crowdQuery, unsigned maxPathRes)
{
	purge();

	m_pathResult = (dtPolyRef*) dtAlloc(sizeof(dtPolyRef) * maxPathRes, DT_ALLOC_PERM);
	m_maxPathRes = maxPathRes;

	if (!m_pathQueue.init(m_maxPathRes, m_maxPathQueueNodes, crowdQuery.getNavMeshQuery()->getAttachedNavMesh()))
		return false;

	if (!m_pathResult)
		return false;

	return true;
}

void dtPathFollowing::purge()
{
	if (m_pathResult)
	{
		dtFree(m_pathResult);
		m_pathResult = 0;
	}

	m_maxPathRes = 0;
	m_maxAgents = 0;
}

void dtPathFollowing::prepare(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, const float dt, 
	const dtPathFollowingParams& currentParam, dtPathFollowingParams& newParam)
{
	checkPathValidity(crowdQuery, oldAgent, newAgent, dt, &newParam);
	updateMoveRequest(crowdQuery, oldAgent, newAgent, currentParam, newParam);
	updateTopologyOptimization(crowdQuery, oldAgent, dt, &newParam);
}

void dtPathFollowing::getVelocity(const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, dtPathFollowingParams* agParams)
{
	if (oldAgent.state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE)
		return;

	float dvel[] = {0, 0, 0};

	if (agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
	{
		dtVcopy(dvel, agParams->targetPos);
	}
	else
	{
		// Calculate steering direction.
		if (oldAgent.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
			calcSmoothSteerDirection(oldAgent, dvel, agParams);
		else
			calcStraightSteerDirection(oldAgent, dvel, agParams);

		// Calculate speed scale, which tells the agent to slowdown at the end of the path.
		const float slowDownRadius = oldAgent.radius * 2;	// TODO: make less hacky.
		float speedScale = 0.f;

		if (slowDownRadius > EPSILON)
			speedScale = getDistanceToGoal(oldAgent, slowDownRadius, agParams) / slowDownRadius;

		dtVscale(dvel, dvel, oldAgent.maxSpeed * speedScale);
	}
	
	dvel[1] = 0;
	dtVcopy(newAgent.desiredVelocity, dvel);
}


float dtPathFollowing::getDistanceToGoal(const dtCrowdAgent& ag, const float range, dtPathFollowingParams* agParams)
{
	if (!agParams->ncorners)
		return range;

	const bool endOfPath = (agParams->cornerFlags[agParams->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	if (endOfPath)
		return dtMin(dtVdist2D(ag.position, &agParams->cornerVerts[(agParams->ncorners-1)*3]), range);

	return range;
}

void dtPathFollowing::calcSmoothSteerDirection(const dtCrowdAgent& ag, float* dir, dtPathFollowingParams* agParams)
{
	if (!agParams->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}

	const int ip0 = 0;
	const int ip1 = dtMin<unsigned>(1, agParams->ncorners-1);
	const float* p0 = &agParams->cornerVerts[ip0*3];
	const float* p1 = &agParams->cornerVerts[ip1*3];

	float dir0[3], dir1[3];
	dtVsub(dir0, p0, ag.position);
	dtVsub(dir1, p1, ag.position);
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

void dtPathFollowing::calcStraightSteerDirection(const dtCrowdAgent& ag, float* dir, dtPathFollowingParams* agParams)
{
	if (!agParams->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &agParams->cornerVerts[0], ag.position);
	dir[1] = 0;
	dtVnormalize(dir);
}

void dtPathFollowing::triggerOffMeshConnections(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, dtPathFollowingParams* agParams)
{
	if (oldAgent.state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE || agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;

	// Check 
	const float triggerRadius = oldAgent.radius * 2.25f;
	if (overOffmeshConnection(oldAgent, triggerRadius, agParams))
	{
		// Adjust the path over the off-mesh connection.
		dtPolyRef refs[2];
		if (agParams->corridor.moveOverOffmeshConnection(agParams->cornerPolys[agParams->ncorners-1], refs,
			newAgent.offmeshStartPos, newAgent.offmeshEndPos, query.getNavMeshQuery()))
		{
			dtVcopy(newAgent.offmeshInitPos, oldAgent.position);

			newAgent.offmeshElaspedTime = 0.0f;
			newAgent.offmeshTotalTime = (dtVdist2D(newAgent.offmeshStartPos, newAgent.offmeshEndPos) / oldAgent.maxSpeed) * 0.5f;

			newAgent.state = DT_CROWDAGENT_STATE_OFFMESH;
			agParams->ncorners = 0;
			return;
		}
		else
		{
			// Path validity check will ensure that bad/blocked connections will be replanned.
		}
	}
}

void dtPathFollowing::getNextCorner(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& ag, dtPathFollowingParams* agParams)
{
	dtCrowdAgentDebugInfo* debug = getBehaviorParams(ag.id)->debugInfos;
	const int debugIdx = debug ? debug->idx : -1;

	if (ag.state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE || agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;


	// Find corners for steering
	agParams->ncorners = agParams->corridor.findCorners(agParams->cornerVerts, agParams->cornerFlags, agParams->cornerPolys,
		dtPathFollowingParams::DT_CROWDAGENT_MAX_CORNERS, crowdQuery.getNavMeshQuery(), crowdQuery.getQueryFilter());

	// Check to see if the corner after the next corner is directly visible,
	// and short cut to there.
	if ((ag.updateFlags & DT_CROWD_OPTIMIZE_VIS) && agParams->ncorners > 0)
	{
		dtPathFollowingParams* params = this->getBehaviorParams(ag.id);
		float pathOptRange = (params) ? params->pathOptimizationRange : 0.f;

		const float* target = &agParams->cornerVerts[dtMin<unsigned>(1,agParams->ncorners-1)*3];
		agParams->corridor.optimizePathVisibility(target, pathOptRange, crowdQuery.getNavMeshQuery(), crowdQuery.getQueryFilter());

		// Copy data for debug purposes.
		if (debugIdx == this->getBehaviorParams(ag.id)->debugIndex)
		{
			dtVcopy(debug->optStart, agParams->corridor.getPos());
			dtVcopy(debug->optEnd, target);
		}
	}
	else
	{
		// Copy data for debug purposes.
		if (debugIdx == getBehaviorParams(ag.id)->debugIndex)
		{
			dtVset(debug->optStart, 0,0,0);
			dtVset(debug->optEnd, 0,0,0);
		}
	}
}

void dtPathFollowing::updateTopologyOptimization(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& ag, const float dt, dtPathFollowingParams* agParams)
{
	const float OPT_TIME_THR = 0.5f; // seconds
	//const int OPT_MAX_AGENTS = 1;
	//dtCrowdAgent* queue[OPT_MAX_AGENTS];
	//int nqueue = 0;

	if (ag.state != DT_CROWDAGENT_STATE_WALKING)
		return;
	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE || agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;
	if ((ag.updateFlags & DT_CROWD_OPTIMIZE_TOPO) == 0)
		return;
	agParams->topologyOptTime += dt;

	if (agParams->topologyOptTime >= OPT_TIME_THR)
    {
        agParams->corridor.optimizePathTopology(const_cast<dtNavMeshQuery*>(crowdQuery.getNavMeshQuery()), crowdQuery.getQueryFilter());
		agParams->topologyOptTime = 0;
    }
}

int dtPathFollowing::addToOptQueue(const dtCrowdAgent& newag, dtCrowdAgent** agents, const unsigned nagents, const unsigned maxAgents)
{
	const dtPathFollowingParams* newAgParams = getBehaviorParams(newag.id);
	if (!newAgParams)
		return 0;

	const dtPathFollowingParams* agParams = getBehaviorParams(agents[nagents-1]->id);
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
			dtPathFollowingParams* pfParams = getBehaviorParams(agents[i]->id);
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

	agents[slot] = const_cast<dtCrowdAgent*>(&newag);

	return dtMin(nagents+1, maxAgents);
}

void dtPathFollowing::checkPathValidity(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, const float dt, dtPathFollowingParams* agParams)
{
	static const int CHECK_LOOKAHEAD = 10;
	static const float TARGET_REPLAN_DELAY = 1.0; // seconds
	
	if (oldAgent.state != DT_CROWDAGENT_STATE_WALKING)
		return;

	if (agParams->targetState == DT_CROWDAGENT_TARGET_NONE || agParams->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		return;


	agParams->targetReplanTime += dt;

	bool replan = false;
	// First check that the current location is valid.
	const int idx = oldAgent.id;
	float agentPos[3];
	dtPolyRef agentRef = agParams->corridor.getFirstPoly();
	dtVcopy(agentPos, oldAgent.position);
	if (!crowdQuery.getNavMeshQuery()->isValidPolyRef(agentRef, crowdQuery.getQueryFilter()))
	{
		// Current location is not valid, try to reposition.
		// TODO: this can snap agents, how to handle that?
		float nearest[3];
		agentRef = 0;
		crowdQuery.getNavMeshQuery()->findNearestPoly(oldAgent.position, crowdQuery.getQueryExtents(), crowdQuery.getQueryFilter(), &agentRef, nearest);
		dtVcopy(agentPos, nearest);

		if (!agentRef)
		{
			// Could not find location in navmesh, set state to invalid.
			agParams->corridor.reset(0, agentPos);
			newAgent.state = DT_CROWDAGENT_STATE_INVALID;
			return;
		}

		// Make sure the first polygon is valid, but leave other valid
		// polygons in the path so that replanner can adjust the path better.
		agParams->corridor.fixPathStart(agentRef, agentPos);
		//			agParams->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
		dtVcopy(newAgent.position, agentPos);

		replan = true;
	}
		
	// Try to recover move request position.
	if (agParams->targetState != DT_CROWDAGENT_TARGET_NONE && agParams->targetState != DT_CROWDAGENT_TARGET_FAILED)
	{
		if (!crowdQuery.getNavMeshQuery()->isValidPolyRef(agParams->targetRef, crowdQuery.getQueryFilter()))
		{
			// Current target is not valid, try to reposition.
			float nearest[3];
			crowdQuery.getNavMeshQuery()->findNearestPoly(agParams->targetPos, crowdQuery.getQueryExtents(), crowdQuery.getQueryFilter(), &agParams->targetRef, nearest);
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
	if (!agParams->corridor.isValid(CHECK_LOOKAHEAD, crowdQuery.getNavMeshQuery(), crowdQuery.getQueryFilter()))
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
/// The request will be processed during the next update.
bool dtPathFollowing::requestMoveTarget(const unsigned idx, dtPolyRef ref, const float* pos)
{
	if (!ref)
		return false;
	
	dtPathFollowingParams* agParams = getBehaviorParams(idx);
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

bool dtPathFollowing::resetMoveTarget(const unsigned idx)
{
	dtPathFollowingParams* agParams = getBehaviorParams(idx);
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

bool dtPathFollowing::requestMoveTargetReplan(const unsigned idx, dtPolyRef ref, const float* pos)
{
	dtPathFollowingParams* agParams = getBehaviorParams(idx);
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

void dtPathFollowing::updateMoveRequest(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& /*newAgent*/, 
	const dtPathFollowingParams& currentParams, dtPathFollowingParams& newParams)
{
	const int PATH_MAX_AGENTS = 8;
	dtCrowdAgent* queue[PATH_MAX_AGENTS];
	int nqueue = 0;

	// Fire off new requests.

	if (oldAgent.state != DT_CROWDAGENT_STATE_INVALID && currentParams.targetState != DT_CROWDAGENT_TARGET_NONE && 
		currentParams.targetState != DT_CROWDAGENT_TARGET_VELOCITY)
	{
		if (currentParams.targetState == DT_CROWDAGENT_TARGET_REQUESTING)
		{
			const dtPolyRef* path = currentParams.corridor.getPath();
			const int npath = currentParams.corridor.getPathCount();
			dtAssert(npath);

			static const int MAX_RES = 32;
			float reqPos[3];
			dtPolyRef reqPath[MAX_RES];	// The path to the request location
			int reqPathCount = 0;

			// Quick search towards the goal.
			static const int MAX_ITER = 20;
			const_cast<dtNavMeshQuery*>(crowdQuery.getNavMeshQuery())->initSlicedFindPath(path[0], currentParams.targetRef, oldAgent.position, 
				currentParams.targetPos, crowdQuery.getQueryFilter());
			const_cast<dtNavMeshQuery*>(crowdQuery.getNavMeshQuery())->updateSlicedFindPath(MAX_ITER, 0);
			dtStatus status = 0;
			if (currentParams.targetReplan) // && npath > 10)
			{
				// Try to use existing steady path during replan if possible.
				status = const_cast<dtNavMeshQuery*>(crowdQuery.getNavMeshQuery())->finalizeSlicedFindPathPartial(path, npath, reqPath, &reqPathCount, MAX_RES);
			}
			else
			{
				// Try to move towards target when goal changes.
				status = const_cast<dtNavMeshQuery*>(crowdQuery.getNavMeshQuery())->finalizeSlicedFindPath(reqPath, &reqPathCount, MAX_RES);
			}

			if (!dtStatusFailed(status) && reqPathCount > 0)
			{
				// In progress or succeed.
				if (reqPath[reqPathCount-1] != currentParams.targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					status = crowdQuery.getNavMeshQuery()->closestPointOnPoly(reqPath[reqPathCount-1], currentParams.targetPos, reqPos);
					if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					dtVcopy(reqPos, currentParams.targetPos);
				}
			}
			else
			{
				reqPathCount = 0;
			}

			if (!reqPathCount)
			{
				// Could not find path, start the request from current location.
				dtVcopy(reqPos, oldAgent.position);
				reqPath[0] = path[0];
				reqPathCount = 1;
			}

			newParams.corridor.setCorridor(reqPos, reqPath, reqPathCount);

			if (reqPath[reqPathCount-1] == currentParams.targetRef)
			{
				newParams.targetState = DT_CROWDAGENT_TARGET_VALID;
				newParams.targetReplanTime = 0.0;
			}
			else
			{
				// The path is longer or potentially unreachable, full plan.
				newParams.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
			}
		}

		if (newParams.targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			nqueue = addToPathQueue(oldAgent, queue, nqueue, PATH_MAX_AGENTS);
		}
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];

		dtPathFollowingParams* pfParams = getBehaviorParams(ag->id);
		if (!pfParams)
			continue;
		if (ag->id == oldAgent.id)
			pfParams = &newParams;

		pfParams->targetPathqRef = m_pathQueue.request(pfParams->corridor.getLastPoly(), pfParams->targetRef,
			pfParams->corridor.getTarget(), pfParams->targetPos, crowdQuery.getQueryFilter());
		if (pfParams->targetPathqRef != DT_PATHQ_INVALID)
			pfParams->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
	}


	// Update requests.
	m_pathQueue.update(m_maxIterPerUpdate);

	dtStatus status;

	// Process path results.

	if (oldAgent.active && currentParams.targetState != DT_CROWDAGENT_TARGET_NONE && currentParams.targetState != DT_CROWDAGENT_TARGET_VELOCITY)
	{
		/*if (currentParams.targetState == DT_CROWDAGENT_TARGET_NONE || currentParams.targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;*/

		if (currentParams.targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			status = m_pathQueue.getRequestStatus(currentParams.targetPathqRef);
			if (dtStatusFailed(status))
			{
				// Path find failed, retry if the target location is still valid.
				newParams.targetPathqRef = DT_PATHQ_INVALID;

				if (currentParams.targetRef)
					newParams.targetState = DT_CROWDAGENT_TARGET_REQUESTING;
				else
					newParams.targetState = DT_CROWDAGENT_TARGET_FAILED;

				newParams.targetReplanTime = 0.0;
			}
			else if (dtStatusSucceed(status))
			{
				const dtPolyRef* path = currentParams.corridor.getPath();
				const int npath = currentParams.corridor.getPathCount();
				dtAssert(npath);

				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, newParams.targetPos);

				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = 0;
				status = m_pathQueue.getPathResult(newParams.targetPathqRef, res, &nres, m_maxPathRes);
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
					if (res[nres-1] != newParams.targetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						float nearest[3];
						status = crowdQuery.getNavMeshQuery()->closestPointOnPoly(res[nres-1], targetPos, nearest);
						if (dtStatusSucceed(status))
							dtVcopy(targetPos, nearest);
						else
							valid = false;
					}
				}

				if (valid)
				{
					// Set current corridor.
					newParams.corridor.setCorridor(targetPos, res, nres);

					newParams.targetState = DT_CROWDAGENT_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
					newParams.targetState = DT_CROWDAGENT_TARGET_FAILED;
				}

				newParams.targetReplanTime = 0.0;
			}
		}
	}
}

int dtPathFollowing::addToPathQueue(const dtCrowdAgent& newag, dtCrowdAgent** agents, const unsigned nagents, const unsigned maxAgents)
{
	const dtPathFollowingParams* pfParams = getBehaviorParams(newag.id);
	if (!pfParams)
		return 0;

	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (pfParams->targetReplanTime <= getBehaviorParams(agents[nagents-1]->id)->targetReplanTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (pfParams->targetReplanTime >= getBehaviorParams(agents[i]->id)->targetReplanTime)
				break;

		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);

		dtAssert(tgt+n <= maxAgents);

		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*)*n);
		slot = i;
	}

	agents[slot] = const_cast<dtCrowdAgent*>(&newag);

	return dtMin(nagents+1, maxAgents);
}

bool dtPathFollowing::overOffmeshConnection(const dtCrowdAgent& ag, const float radius, dtPathFollowingParams* agParams)
{
	if (!agParams->ncorners)
		return false;

	const bool offMeshConnection = (agParams->cornerFlags[agParams->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (offMeshConnection)
	{
		const float distSq = dtVdist2DSqr(ag.position, &agParams->cornerVerts[(agParams->ncorners-1)*3]);
		if (distSq < radius*radius)
			return true;
	}

	return false;
}

void dtPathFollowing::doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
	const dtPathFollowingParams& currentParams, dtPathFollowingParams& newParams, float dt)
{
	// Update the corridor
	newParams.corridor.movePosition(oldAgent.position, query.getNavMeshQuery(), query.getQueryFilter());
	
	prepare(query, oldAgent, newAgent, dt, currentParams, newParams);
	getNextCorner(query, oldAgent, &newParams);
	triggerOffMeshConnections(query, oldAgent, newAgent, &newParams);
	getVelocity(oldAgent, newAgent, &newParams);
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
}

void dtPathFollowingParams::init(unsigned maxPathResults)
{
	corridor.init(maxPathResults);
}
