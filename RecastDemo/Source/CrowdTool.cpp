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
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "CrowdTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "DetourCrowd.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"
#include "DetourNode.h"
#include "DetourPathFollowing.h"
#include "SampleInterfaces.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	dtVsub(d, sq, sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
	
	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) dtSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

static void getAgentBounds(const dtCrowdAgent* ag, float* bmin, float* bmax)
{
	const float* p = ag->position;
	const float r = ag->radius;
	const float h = ag->height;
	bmin[0] = p[0] - r;
	bmin[1] = p[1];
	bmin[2] = p[2] - r;
	bmax[0] = p[0] + r;
	bmax[1] = p[1] + h;
	bmax[2] = p[2] + r;
}

CrowdToolState::CrowdToolState() :
	m_sample(0),
	m_nav(0),
	m_crowd(0),
	m_targetRef(0),
	m_run(true),
	m_pf(0)
{
	m_toolParams.m_expandSelectedDebugDraw = true;
	m_toolParams.m_showCorners = false;
	m_toolParams.m_showCollisionSegments = false;
	m_toolParams.m_showPath = false;
	m_toolParams.m_showVO = false;
	m_toolParams.m_showOpt = false;
	m_toolParams.m_showNeis = false;
	m_toolParams.m_expandDebugDraw = false;
	m_toolParams.m_showLabels = false;
	m_toolParams.m_showGrid = false;
	m_toolParams.m_showNodes = false;
	m_toolParams.m_showPerfGraph = false;
	m_toolParams.m_showDetailAll = false;
	m_toolParams.m_expandOptions = true;
	m_toolParams.m_anticipateTurns = true;
	m_toolParams.m_optimizeVis = true;
	m_toolParams.m_optimizeTopo = true;
	m_toolParams.m_obstacleAvoidance = true;
	m_toolParams.m_obstacleAvoidanceType = 3.0f;
	m_toolParams.m_separation = false;
	m_toolParams.m_separationWeight = 2.0f;
	
	memset(m_trails, 0, sizeof(m_trails));
	
	m_vod = dtAllocObstacleAvoidanceDebugData();
	m_vod->init(2048);
}

CrowdToolState::~CrowdToolState()
{
	dtFreeObstacleAvoidanceDebugData(m_vod);
}

void CrowdToolState::init(class Sample* sample)
{
	if (m_sample != sample)
	{
		m_sample = sample;
//		m_oldFlags = m_sample->getNavMeshDrawFlags();
//		m_sample->setNavMeshDrawFlags(m_oldFlags & ~DU_DRAWNAVMESH_CLOSEDLIST);
	}
	
	dtNavMesh* nav = m_sample->getNavMesh();
	dtCrowd* crowd = m_sample->getCrowd();
	
	if (nav && crowd && (m_nav != nav || m_crowd != crowd))
	{
		m_nav = nav;
		m_crowd = crowd;
	
		crowd->init(MAX_AGENTS, m_sample->getAgentRadius(), nav);
		
		// Make polygons with 'disabled' flag invalid.
		crowd->getCrowdQuery()->getQueryFilter()->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);

		m_pf = dtPathFollowing::allocate(m_crowd->getAgentCount());
		m_pf->init(*crowd->getCrowdQuery());
		
		//// Setup local avoidance params to different qualities.
		//dtObstacleAvoidanceParams params;
		//// Use mostly default settings, copy from dtCrowd.
		//memcpy(&params, crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));
		//
		//// Low (11)
		//params.velBias = 0.5f;
		//params.adaptiveDivs = 5;
		//params.adaptiveRings = 2;
		//params.adaptiveDepth = 1;
		//crowd->setObstacleAvoidanceParams(0, &params);
		//
		//// Medium (22)
		//params.velBias = 0.5f;
		//params.adaptiveDivs = 5; 
		//params.adaptiveRings = 2;
		//params.adaptiveDepth = 2;
		//crowd->setObstacleAvoidanceParams(1, &params);
		//
		//// Good (45)
		//params.velBias = 0.5f;
		//params.adaptiveDivs = 7;
		//params.adaptiveRings = 2;
		//params.adaptiveDepth = 3;
		//crowd->setObstacleAvoidanceParams(2, &params);
		//
		//// High (66)
		//params.velBias = 0.5f;
		//params.adaptiveDivs = 7;
		//params.adaptiveRings = 3;
		//params.adaptiveDepth = 3;
		//
		//crowd->setObstacleAvoidanceParams(3, &params);
	}
}

void CrowdToolState::reset()
{
}

void CrowdToolState::handleRender()
{
	DebugDrawGL dd;
	const float rad = m_sample->getAgentRadius();
	
	dtNavMesh* nav = m_sample->getNavMesh();
	dtCrowd* crowd = m_sample->getCrowd();

	if (!nav || !crowd)
		return;
	
	if (m_toolParams.m_showNodes)
	{
		const dtNavMeshQuery* navquery = crowd->getCrowdQuery()->getNavMeshQuery();
		if (navquery)
			duDebugDrawNavMeshNodes(&dd, *navquery);
	}

	dd.depthMask(false);
	
	// Draw paths
	//if (m_toolParams.m_showPath)
	//{
	//	for (int i = 0; i < crowd->getAgentCount(); i++)
	//	{
	//		if (m_toolParams.m_showDetailAll == false && i != m_agentDebug.idx)
	//			continue;
	//		const dtCrowdAgent* ag =crowd->getAgent(i);
	//		if (!ag->active)
	//			continue;
	//		const dtPolyRef* path = ag->corridor.getPath();
	//		const int npath = ag->corridor.getPathCount();			
	//		for (int j = 0; j < npath; ++j)
	//			duDebugDrawNavMeshPoly(&dd, *nav, path[j], duRGBA(255,255,255,24));
	//	}
	//}
	
	if (m_targetRef)
		duDebugDrawCross(&dd, m_targetPos[0],m_targetPos[1]+0.1f,m_targetPos[2], rad, duRGBA(255,255,255,192), 2.0f);
	
	// Occupancy grid.
	if (m_toolParams.m_showGrid)
	{
		float gridy = -FLT_MAX;
		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active) continue;
			const float* pos = ag->position;
			gridy = dtMax(gridy, pos[1]);
		}
		gridy += 1.0f;
	}
	
	// Trail
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const AgentTrail* trail = &m_trails[i];
		const float* pos = ag->position;
		
		dd.begin(DU_DRAW_LINES,3.0f);
		float prev[3], preva = 1;
		dtVcopy(prev, pos);
		for (int j = 0; j < AGENT_MAX_TRAIL-1; ++j)
		{
			const int idx = (trail->htrail + AGENT_MAX_TRAIL-j) % AGENT_MAX_TRAIL;
			const float* v = &trail->trail[idx*3];
			float a = 1 - j/(float)AGENT_MAX_TRAIL;
			dd.vertex(prev[0],prev[1]+0.1f,prev[2], duRGBA(0,0,0,(int)(128*preva)));
			dd.vertex(v[0],v[1]+0.1f,v[2], duRGBA(0,0,0,(int)(128*a)));
			preva = a;
			dtVcopy(prev, v);
		}
		dd.end();
	}
	
	// Corners & co
	for (int i = 0; i < crowd->getAgentCount(); i++)
	{
		if (m_toolParams.m_showDetailAll == false)
			continue;
		const dtCrowdAgent* ag =crowd->getAgent(i);
		if (!ag->active)
			continue;
			
		const float radius = ag->radius;
		const float* pos = ag->position;
		
		if (m_toolParams.m_showCollisionSegments)
		{
			const float* center = m_crowd->getAgentEnvironment(ag->id)->boundary.getCenter();
			duDebugDrawCross(&dd, center[0],center[1]+radius,center[2], 0.2f, duRGBA(192,0,128,255), 2.0f);
			duDebugDrawCircle(&dd, center[0],center[1]+radius,center[2], ag->perceptionDistance,
							  duRGBA(192,0,128,128), 2.0f);
			
			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int j = 0; j <m_crowd->getAgentEnvironment(ag->id)->boundary.getSegmentCount(); ++j)
			{
				const float* s = m_crowd->getAgentEnvironment(ag->id)->boundary.getSegment(j);
				unsigned int col = duRGBA(192,0,128,192);
				if (dtTriArea2D(pos, s, s+3) < 0.0f)
					col = duDarkenCol(col);
				
				duAppendArrow(&dd, s[0],s[1]+0.2f,s[2], s[3],s[4]+0.2f,s[5], 0.0f, 0.3f, col);
			}
			dd.end();
		}
		
		if (m_toolParams.m_showNeis)
		{
			duDebugDrawCircle(&dd, pos[0],pos[1]+radius,pos[2], ag->perceptionDistance,
							  duRGBA(0,192,128,128), 2.0f);
			
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int j = 0; j < m_crowd->getAgentEnvironment(ag->id)->nbNeighbors; ++j)
			{
				// Get 'n'th active agent.
				// TODO: fix this properly.
				const dtCrowdAgent* nei = crowd->getAgent(m_crowd->getAgentEnvironment(ag->id)->neighbors[j].idx);
				if (nei)
				{
					dd.vertex(pos[0],pos[1]+radius,pos[2], duRGBA(0,192,128,128));
					dd.vertex(nei->position[0],nei->position[1]+radius,nei->position[2], duRGBA(0,192,128,128));
				}
			}
			dd.end();
		}
	}
	
	// Agent cylinders.
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const float radius = ag->radius;
		const float* pos = ag->position;
		
		unsigned int col = duRGBA(0,0,0,32);
		//if (m_agentDebug.idx == i)
		//	col = duRGBA(255,0,0,128);
			
		duDebugDrawCircle(&dd, pos[0], pos[1], pos[2], radius, col, 2.0f);
	}
	
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const float height = ag->height;
		const float radius = ag->radius;
		const float* pos = ag->position;
		
		unsigned int col = duRGBA(220,220,220,128);
		
		duDebugDrawCylinder(&dd, pos[0]-radius, pos[1]+radius*0.1f, pos[2]-radius,
							pos[0]+radius, pos[1]+height, pos[2]+radius, col);
	}
	
	
	if (m_toolParams.m_showVO)
	{
		for (int i = 0; i < crowd->getAgentCount(); i++)
		{
			if (m_toolParams.m_showDetailAll == false)
				continue;
			const dtCrowdAgent* ag =crowd->getAgent(i);
			if (!ag->active)
				continue;
		
			// Draw detail about agent sela
			//const dtObstacleAvoidanceDebugData* vod = m_agentDebug.vod;
			
			const float dx = ag->position[0];
			const float dy = ag->position[1]+ag->height;
			const float dz = ag->position[2];
			
			duDebugDrawCircle(&dd, dx,dy,dz, ag->maxSpeed, duRGBA(255,255,255,64), 2.0f);
		}
	}
	
	// Velocity stuff.
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const float radius = ag->radius;
		const float height = ag->height;
		const float* pos = ag->position;
		const float* vel = ag->velocity;
		const float* dvel = ag->desiredVelocity;
		
		unsigned int col = duRGBA(220,220,220,192);
		
		duDebugDrawCircle(&dd, pos[0], pos[1]+height, pos[2], radius, col, 2.0f);
		
		duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
						 pos[0]+dvel[0],pos[1]+height+dvel[1],pos[2]+dvel[2],
						 0.0f, 0.4f, duRGBA(0,192,255,192), 1.0f);
		
		duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
						 pos[0]+vel[0],pos[1]+height+vel[1],pos[2]+vel[2],
						 0.0f, 0.4f, duRGBA(0,0,0,160), 2.0f);
	}
	
	dd.depthMask(true);
}


void CrowdToolState::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_targetRef && gluProject((GLdouble)m_targetPos[0], (GLdouble)m_targetPos[1], (GLdouble)m_targetPos[2],
								  model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y+25), IMGUI_ALIGN_CENTER, "TARGET", imguiRGBA(0,0,0,220));
	}
	
	char label[32];
	
	if (m_toolParams.m_showNodes)
	{
		dtCrowd* crowd = m_sample->getCrowd();
		if (crowd)
		{
			const dtNavMeshQuery* navquery = crowd->getCrowdQuery()->getNavMeshQuery();
			const dtNodePool* pool = navquery->getNodePool();
			if (pool)
			{
				const float off = 0.5f;
				for (int i = 0; i < pool->getHashSize(); ++i)
				{
					for (dtNodeIndex j = pool->getFirst(i); j != DT_NULL_IDX; j = pool->getNext(j))
					{
						const dtNode* node = pool->getNodeAtIdx(j+1);
						if (!node) continue;

						if (gluProject((GLdouble)node->pos[0],(GLdouble)node->pos[1]+off,(GLdouble)node->pos[2],
									   model, proj, view, &x, &y, &z))
						{
							const float heuristic = node->total;// - node->cost;
							snprintf(label, 32, "%.2f", heuristic);
							imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(0,0,0,220));
						}
					}
				}
			}
		}
	}
	
	if (m_toolParams.m_showLabels)
	{
		dtCrowd* crowd = m_sample->getCrowd();
		if (crowd)
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = crowd->getAgent(i);
				if (!ag->active) continue;
				const float* pos = ag->position;
				const float h = ag->height;
				if (gluProject((GLdouble)pos[0], (GLdouble)pos[1]+h, (GLdouble)pos[2],
							   model, proj, view, &x, &y, &z))
				{
					snprintf(label, 32, "%d", i);
					imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(0,0,0,220));
				}
			}			
		}
	}
	
	if (m_toolParams.m_showPerfGraph)
	{
		GraphParams gp;
		gp.setRect(300, 10, 500, 200, 8);
		gp.setValueRange(0.0f, 2.0f, 4, "ms");
		
		drawGraphBackground(&gp);
		drawGraph(&gp, &m_crowdTotalTime, 1, "Total", duRGBA(255,128,0,255));
		
		gp.setRect(300, 10, 500, 50, 8);
		gp.setValueRange(0.0f, 2000.0f, 1, "");
		drawGraph(&gp, &m_crowdSampleCount, 0, "Sample Count", duRGBA(96,96,96,128));
	}
	
}

void CrowdToolState::handleUpdate(const float dt)
{
	if (m_run)
		updateTick(dt);
}

void CrowdToolState::addAgent(const float* p)
{
	if (!m_sample) return;
	dtCrowd* crowd = m_sample->getCrowd();	
	dtCrowdAgent ag;

	if (crowd->addAgent(ag, p))
	{
		ag.radius = m_sample->getAgentRadius();
		ag.height = m_sample->getAgentHeight();
		ag.maxAcceleration = 8.0f;
		ag.maxSpeed = 3.5f;
		ag.updateFlags = 0; 

		if (m_toolParams.m_anticipateTurns)
			ag.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
		if (m_toolParams.m_optimizeVis)
			ag.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
		if (m_toolParams.m_optimizeTopo)
			ag.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
		if (m_toolParams.m_obstacleAvoidance)
			ag.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
		if (m_toolParams.m_separation)
			ag.updateFlags |= DT_CROWD_SEPARATION;
		
		crowd->applyAgent(ag);

		// Init trail
		AgentTrail* trail = &m_trails[ag.id];

		for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
			dtVcopy(&trail->trail[i * 3], p);

		trail->htrail = 0;
	}
}

void CrowdToolState::removeAgent(const int idx)
{
	if (!m_sample) return;
	dtCrowd* crowd = m_sample->getCrowd();

	crowd->removeAgent(idx);
	
	//if (idx == m_agentDebug.idx)
	//	m_agentDebug.idx = -1;
}

void CrowdToolState::hilightAgent(const int idx)
{
	//m_agentDebug.idx = idx;
}

static void calcVel(float* vel, const float* pos, const float* tgt, const float speed)
{
	dtVsub(vel, tgt, pos);
	vel[1] = 0.0;
	dtVnormalize(vel);
	dtVscale(vel, vel, speed);
}

void CrowdToolState::setMoveTarget(const float* p, bool adjust)
{
	if (!m_sample) return;
	
	// Find nearest point on navmesh and set move request to that location.
	dtNavMeshQuery* navquery = m_sample->getNavMeshQuery();
	dtCrowd* crowd = m_sample->getCrowd();
	const dtQueryFilter* filter = crowd->getCrowdQuery()->getQueryFilter();
	const float* ext = crowd->getCrowdQuery()->getQueryExtents();

	if (adjust)
	{
		float vel[3];

		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active) continue;
			calcVel(vel, ag->position, p, ag->maxSpeed);
			//m_pf->requestMoveVelocity(ag->id, vel);
		}
	}
	else
	{
		navquery->findNearestPoly(p, ext, filter, &m_targetRef, m_targetPos);

		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active) continue;
			m_pf->requestMoveTarget(ag->id, m_targetRef, m_targetPos);
		}
	}
}

int CrowdToolState::hitTestAgents(const float* s, const float* p)
{
	if (!m_sample) return -1;
	dtCrowd* crowd = m_sample->getCrowd();
	
	int isel = -1;
	float tsel = FLT_MAX;

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		float bmin[3], bmax[3];
		getAgentBounds(ag, bmin, bmax);
		float tmin, tmax;
		if (isectSegAABB(s, p, bmin,bmax, tmin, tmax))
		{
			if (tmin > 0 && tmin < tsel)
			{
				isel = i;
				tsel = tmin;
			} 
		}
	}

	return isel;
}

void CrowdToolState::updateAgentParams()
{
	if (!m_sample) return;
	dtCrowd* crowd = m_sample->getCrowd();
	if (!crowd) return;
	
	unsigned char updateFlags = 0;
	unsigned char obstacleAvoidanceType = 0;
	
	if (m_toolParams.m_anticipateTurns)
		updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	if (m_toolParams.m_optimizeVis)
		updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	if (m_toolParams.m_optimizeTopo)
		updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	if (m_toolParams.m_obstacleAvoidance)
		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_toolParams.m_obstacleAvoidance)
		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_toolParams.m_separation)
		updateFlags |= DT_CROWD_SEPARATION;
	
	obstacleAvoidanceType = (unsigned char)m_toolParams.m_obstacleAvoidanceType;
	
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		dtCrowdAgent ag;
		crowd->fetchAgent(ag, i);

		if (!ag.active) 
			continue;

		ag.updateFlags = updateFlags;
		crowd->applyAgent(ag);
	}	
}

void CrowdToolState::updateTick(const float dt)
{
	if (!m_sample) return;
	dtNavMesh* nav = m_sample->getNavMesh();
	dtCrowd* crowd = m_sample->getCrowd();
	if (!nav || !crowd) return;
	
	TimeVal startTime = getPerfTime();
	
	crowd->update(dt);
	
	TimeVal endTime = getPerfTime();
	
	// Update agent trails
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		AgentTrail* trail = &m_trails[i];
		if (!ag->active)
			continue;
		// Update agent movement trail.
		trail->htrail = (trail->htrail + 1) % AGENT_MAX_TRAIL;
		dtVcopy(&trail->trail[trail->htrail*3], ag->position);
	}
	
	//m_agentDebug.vod->normalizeSamples();
	
	//m_crowdSampleCount.addSample((float)crowd->getVelocitySampleCount());
	m_crowdTotalTime.addSample(getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
}




CrowdTool::CrowdTool() :
	m_sample(0),
	m_state(0),
	m_mode(TOOLMODE_CREATE)
{
}

CrowdTool::~CrowdTool()
{
}

void CrowdTool::init(Sample* sample)
{
	if (m_sample != sample)
	{
		m_sample = sample;
	}
	
	if (!sample)
		return;
		
	m_state = (CrowdToolState*)sample->getToolState(type());
	if (!m_state)
	{
		m_state = new CrowdToolState();
		sample->setToolState(type(), m_state);
	}
	m_state->init(sample);
}

void CrowdTool::reset()
{	
}

void CrowdTool::handleMenu()
{
	if (!m_state)
		return;
	CrowdToolParams* params = m_state->getToolParams();

	if (imguiCheck("Create Agents", m_mode == TOOLMODE_CREATE))
		m_mode = TOOLMODE_CREATE;
	if (imguiCheck("Move Target", m_mode == TOOLMODE_MOVE_TARGET))
		m_mode = TOOLMODE_MOVE_TARGET;
	if (imguiCheck("Select Agent", m_mode == TOOLMODE_SELECT))
		m_mode = TOOLMODE_SELECT;
	if (imguiCheck("Toggle Polys", m_mode == TOOLMODE_TOGGLE_POLYS))
		m_mode = TOOLMODE_TOGGLE_POLYS;
	
	imguiSeparatorLine();
		
	if (imguiCollapse("Options", 0, params->m_expandOptions))
		params->m_expandOptions = !params->m_expandOptions;
	
	if (params->m_expandOptions)
	{
		imguiIndent();
		if (imguiCheck("Optimize Visibility", params->m_optimizeVis))
		{
			params->m_optimizeVis = !params->m_optimizeVis;
			m_state->updateAgentParams();
		}
		if (imguiCheck("Optimize Topology", params->m_optimizeTopo))
		{
			params->m_optimizeTopo = !params->m_optimizeTopo;
			m_state->updateAgentParams();
		}
		if (imguiCheck("Anticipate Turns", params->m_anticipateTurns))
		{
			params->m_anticipateTurns = !params->m_anticipateTurns;
			m_state->updateAgentParams();
		}
		if (imguiCheck("Obstacle Avoidance", params->m_obstacleAvoidance))
		{
			params->m_obstacleAvoidance = !params->m_obstacleAvoidance;
			m_state->updateAgentParams();
		}
		if (imguiSlider("Avoidance Quality", &params->m_obstacleAvoidanceType, 0.0f, 3.0f, 1.0f))
		{
			m_state->updateAgentParams();
		}
		if (imguiCheck("Separation", params->m_separation))
		{
			params->m_separation = !params->m_separation;
			m_state->updateAgentParams();
		}
		if (imguiSlider("Separation Weight", &params->m_separationWeight, 0.0f, 20.0f, 0.01f))
		{
			m_state->updateAgentParams();
		}
		
		imguiUnindent();
	}

	if (imguiCollapse("Selected Debug Draw", 0, params->m_expandSelectedDebugDraw))
		params->m_expandSelectedDebugDraw = !params->m_expandSelectedDebugDraw;
		
	if (params->m_expandSelectedDebugDraw)
	{
		imguiIndent();
		if (imguiCheck("Show Corners", params->m_showCorners))
			params->m_showCorners = !params->m_showCorners;
		if (imguiCheck("Show Collision Segs", params->m_showCollisionSegments))
			params->m_showCollisionSegments = !params->m_showCollisionSegments;
		if (imguiCheck("Show Path", params->m_showPath))
			params->m_showPath = !params->m_showPath;
		if (imguiCheck("Show VO", params->m_showVO))
			params->m_showVO = !params->m_showVO;
		if (imguiCheck("Show Path Optimization", params->m_showOpt))
			params->m_showOpt = !params->m_showOpt;
		if (imguiCheck("Show Neighbours", params->m_showNeis))
			params->m_showNeis = !params->m_showNeis;
		imguiUnindent();
	}
		
	if (imguiCollapse("Debug Draw", 0, params->m_expandDebugDraw))
		params->m_expandDebugDraw = !params->m_expandDebugDraw;
	
	if (params->m_expandDebugDraw)
	{
		imguiIndent();
		if (imguiCheck("Show Labels", params->m_showLabels))
			params->m_showLabels = !params->m_showLabels;
		if (imguiCheck("Show Prox Grid", params->m_showGrid))
			params->m_showGrid = !params->m_showGrid;
		if (imguiCheck("Show Nodes", params->m_showNodes))
			params->m_showNodes = !params->m_showNodes;
		if (imguiCheck("Show Perf Graph", params->m_showPerfGraph))
			params->m_showPerfGraph = !params->m_showPerfGraph;
		if (imguiCheck("Show Detail All", params->m_showDetailAll))
			params->m_showDetailAll = !params->m_showDetailAll;
		imguiUnindent();
	}
}

void CrowdTool::handleClick(const float* s, const float* p, bool shift)
{
	if (!m_sample) return;
	if (!m_state) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
	dtCrowd* crowd = m_sample->getCrowd();
	if (!crowd) return;

	if (m_mode == TOOLMODE_CREATE)
	{
		if (shift)
		{
			// Delete
			int ahit = m_state->hitTestAgents(s,p);
			if (ahit != -1)
				m_state->removeAgent(ahit);
		}
		else
		{
			// Add
			m_state->addAgent(p);
		}
	}
	else if (m_mode == TOOLMODE_MOVE_TARGET)
	{
		m_state->setMoveTarget(p, shift);
	}
	else if (m_mode == TOOLMODE_SELECT)
	{
		// Highlight
		int ahit = m_state->hitTestAgents(s,p);
		m_state->hilightAgent(ahit);
	}
	else if (m_mode == TOOLMODE_TOGGLE_POLYS)
	{
		dtNavMesh* nav = m_sample->getNavMesh();
		dtNavMeshQuery* navquery = m_sample->getNavMeshQuery();
		if (nav && navquery)
		{
			dtQueryFilter filter;
			const float* ext = crowd->getCrowdQuery()->getQueryExtents();
			float tgt[3];
			dtPolyRef ref;
			navquery->findNearestPoly(p, ext, &filter, &ref, tgt);
			if (ref)
			{
				unsigned short flags = 0;
				if (dtStatusSucceed(nav->getPolyFlags(ref, &flags)))
				{
					flags ^= SAMPLE_POLYFLAGS_DISABLED;
					nav->setPolyFlags(ref, flags);
				}
			}
		}
	}

}

void CrowdTool::handleStep()
{
	if (!m_state) return;
	
	const float dt = 1.0f/20.0f;
	m_state->updateTick(dt);

	m_state->setRunning(false);
}

void CrowdTool::handleToggle()
{
	if (!m_state) return;
	m_state->setRunning(!m_state->isRunning());
}

void CrowdTool::handleUpdate(const float dt)
{
}

void CrowdTool::handleRender()
{
}

void CrowdTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	// Tool help
	const int h = view[3];
	int ty = h-40;
	
	if (m_mode == TOOLMODE_CREATE)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: add agent.  Shift+LMB: remove agent.", imguiRGBA(255,255,255,192));	
	}
	else if (m_mode == TOOLMODE_MOVE_TARGET)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: set move target.  Shift+LMB: adjust set velocity.", imguiRGBA(255,255,255,192));	
		ty -= 20;
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Setting velocity will move the agents without pathfinder.", imguiRGBA(255,255,255,192));	
	}
	else if (m_mode == TOOLMODE_SELECT)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: select agent.", imguiRGBA(255,255,255,192));	
	}
	ty -= 20;
	imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "SPACE: Run/Pause simulation.  1: Step simulation.", imguiRGBA(255,255,255,192));	
	ty -= 20;

	if (m_state && m_state->isRunning())
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- RUNNING -", imguiRGBA(255,32,16,255));	
	else 
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- PAUSED -", imguiRGBA(255,255,255,128));	
}
