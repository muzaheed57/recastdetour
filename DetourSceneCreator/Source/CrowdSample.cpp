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

#include "CrowdSample.h"

#include "InputGeom.h"
#include "NavMeshCreator.h"
#include "JSON.h"

#include "DetourParametrizedBehavior.h"
#include "DetourAlignmentBehavior.h"
#include "DetourCohesionBehavior.h"
#include "DetourCollisionAvoidance.h"
#include "DetourPathFollowing.h"
#include "DetourPipelineBehavior.h"
#include "DetourSeekBehavior.h"
#include "DetourSeparationBehavior.h"

#include <Recast.h>

#include <cstdio>
#include <cstring>

CrowdSample::CrowdSample()
: m_root(0)
, m_agentCfgs()
, m_agentCount(0)
, m_context(0)
, m_maxRadius(0.f)
, m_sceneFileName()
, m_navMesh(0)
{
    strcpy(m_sceneFileName,"");
}

CrowdSample::~CrowdSample()
{
}

char* CrowdSample::getSceneFile()
{
	if (m_root)
	{
		JSONValue* scene = m_root->Child(L"scene");
		if (scene && scene->IsObject())
		{
			JSONValue* file = scene->Child(L"file");
			if (file && file->IsString())
				wcstombs(m_sceneFileName,file->AsString().c_str(), maxPathLen);
		}
	}

	return m_sceneFileName;
}

void CrowdSample::parseAgentsInfo()
{
	JSONValue* agents = m_root->Child(L"agents");
	if (agents && agents->IsArray())
	{
		m_agentCount = rcMin<int>(agents->CountChildren(),maxAgentCount);
		for (std::size_t iAgent(0) ; iAgent < (size_t)m_agentCount ; ++iAgent)
		{
			JSONValue* agent = agents->Child(iAgent);
			if (agent && agent->IsObject())
			{
				JSONValue* parameters = agent->Child(L"parameters");
				if (parameters && parameters->IsObject())
				{
					JSONValue* radius = parameters->Child(L"radius");
					if (radius && radius->IsNumber())
						m_maxRadius = rcMax(m_maxRadius, (float) radius->AsNumber());
				}
			}
		}
	}
}

void CrowdSample::parseCrowd(dtCrowd* crowd)
{
	JSONValue* flocking = m_root->Child(L"flockings");

	if (flocking && flocking->IsArray())
	{
		int nbFlocks = flocking->CountChildren();
		m_flockingsGroups.resize(nbFlocks);

		for (std::size_t i(0) ; i < (size_t)nbFlocks ; ++i)
		{
			JSONValue* flock = flocking->Child(i);

			if (flock && flock->IsObject())
			{
				JSONValue* desiredSeparation = flock->Child(L"desiredSeparation");
				if (desiredSeparation && desiredSeparation->IsNumber())
					m_flockingsGroups.at(i).desiredSeparation = (float) desiredSeparation->AsNumber();

				JSONValue* separationWeight = flock->Child(L"separationWeight");
				if (separationWeight && separationWeight->IsNumber())
					m_flockingsGroups.at(i).separationWeight = (float) separationWeight->AsNumber();

				JSONValue* cohesionWeight = flock->Child(L"cohesionWeight");
				if (cohesionWeight && cohesionWeight->IsNumber())
					m_flockingsGroups.at(i).cohesionWeight = (float) cohesionWeight->AsNumber();

				JSONValue* alignmentWeight = flock->Child(L"alignmentWeight");
				if (alignmentWeight && alignmentWeight->IsNumber())
					m_flockingsGroups.at(i).alignmentWeight = (float) alignmentWeight->AsNumber();
			}
		}
	}

	JSONValue* agents = m_root->Child(L"agents");
	if (agents && agents->IsArray())
	{
		m_agentCount = rcMin<int>(agents->CountChildren(),maxAgentCount);
		for (std::size_t iAgent(0) ; iAgent < (size_t)m_agentCount ; ++iAgent)
		{
			memset(&m_agentCfgs[iAgent],0,sizeof(m_agentCfgs[iAgent]));
			JSONValue* agent = agents->Child(iAgent);
			if (agent && agent->IsObject())
			{
				JSONValue* position = agent->Child(L"position");
				if (position && position->IsArray())
				{
					m_agentCfgs[iAgent].position[0] = (float)position->Child((size_t)0)->AsNumber();
					m_agentCfgs[iAgent].position[1] = (float)position->Child(1)->AsNumber();
					m_agentCfgs[iAgent].position[2] = (float)position->Child(2)->AsNumber();
				}
				
				JSONValue* parameters = agent->Child(L"parameters");
				if (parameters && parameters->IsObject())
				{
					JSONValue* maxSpeed = parameters->Child(L"maxSpeed");
					if (maxSpeed && maxSpeed->IsNumber())
						m_agentCfgs[iAgent].maxSpeed= (float)maxSpeed->AsNumber();

					JSONValue* maxAcceleration = parameters->Child(L"maxAcceleration");
					if (maxAcceleration && maxAcceleration->IsNumber())
						m_agentCfgs[iAgent].maxAcceleration = (float)maxAcceleration->AsNumber();

					JSONValue* radius = parameters->Child(L"radius");
					if (radius && radius->IsNumber())
						m_agentCfgs[iAgent].radius = (float)radius->AsNumber();

					JSONValue* height = parameters->Child(L"height");
					if (height && height->IsNumber())
						m_agentCfgs[iAgent].height = (float)height->AsNumber();

					JSONValue* collisionQueryRange = parameters->Child(L"collisionQueryRange");
					if (collisionQueryRange && collisionQueryRange->IsNumber())
						m_agentCfgs[iAgent].collisionQueryRange = (float)collisionQueryRange->AsNumber();

					JSONValue* pipeline = parameters->Child(L"pipeline");
					if (pipeline && pipeline->IsArray())
						parsePipeline(pipeline, iAgent, crowd);

					JSONValue* behavior = parameters->Child(L"behavior");
					if (behavior && behavior->IsObject())
						parseBehavior(behavior, iAgent, crowd, false);

					JSONValue* updateFlags = parameters->Child(L"updateFlags");
					if (updateFlags && updateFlags->IsArray())
					{
						for (std::size_t iFlag(0), size(updateFlags->CountChildren()) ; iFlag < size ; ++iFlag)
						{
							JSONValue* updateFlag = updateFlags->Child(iFlag);

							if (updateFlag && updateFlag->IsString())
							{
								const std::wstring& updateFlagStr = updateFlag->AsString();

								if (updateFlagStr==L"DT_CROWD_ANTICIPATE_TURNS")
									m_agentCfgs[iAgent].updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
								else if (updateFlagStr==L"DT_CROWD_OBSTACLE_AVOIDANCE")
									m_agentCfgs[iAgent].updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
								else if (updateFlagStr==L"DT_CROWD_SEPARATION")
									m_agentCfgs[iAgent].updateFlags |= DT_CROWD_SEPARATION;
								else if (updateFlagStr==L"DT_CROWD_OPTIMIZE_VIS")
									m_agentCfgs[iAgent].updateFlags |= DT_CROWD_OPTIMIZE_VIS;
								else if (updateFlagStr==L"DT_CROWD_OPTIMIZE_TOPO")
									m_agentCfgs[iAgent].updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
							}
						}
					}
				}
			}
		}
	}
}

bool CrowdSample::loadFromBuffer( const char* data )
{
	m_root = JSON::Parse(data);

    if (!m_root)
    {
        //Unable to parse the JSON data.
        return false;
    }
        
	return true;
}

void CrowdSample::parsePipeline(JSONValue* pipeline, std::size_t iAgent, dtCrowd* crowd)
{
	if (pipeline && pipeline->IsArray())
	{
		for (std::size_t iFlag(0), size(pipeline->CountChildren()) ; iFlag < size ; ++iFlag)
		{
			JSONValue* child = pipeline->Child(iFlag);
			JSONValue* behavior = child->Child(L"behavior");
			JSONValue* pipe = child->Child(L"pipeline");

			if (behavior && behavior->IsObject())
				parseBehavior(behavior, iAgent, crowd, true);
			else if (pipe && pipe->IsArray())
				parsePipeline(pipe, iAgent, crowd);
		}
	}
}

void CrowdSample::parseBehavior(JSONValue* behavior, std::size_t iAgent, dtCrowd* crowd, bool pipeline)
{
	JSONValue* type = behavior->Child(L"type");

	// Collision Avoidance behavior
	if (type && type->IsString() && type->AsString() == L"collisionAvoidance")
	{
		dtCollisionAvoidance* ca = new dtCollisionAvoidance(m_agentCount, crowd->getAgentsEnvironment());
		ca->init();

		dtObstacleAvoidanceParams* params = new dtObstacleAvoidanceParams;

		params = ca->addBehaviorParams(*crowd->getAgent(iAgent));

		if (params)
		{
			params->caAgents = crowd->getAgents();
			params->caDebug = 0;
			params->velBias = 0.4f;
			params->weightDesVel = 2.0f;
			params->weightCurVel = 0.75f;
			params->weightSide = 0.75f;
			params->weightToi = 2.5f;
			params->horizTime = 2.5f;
			params->gridSize = 33;
			params->adaptiveDivs = 7;
			params->adaptiveRings = 2;
			params->adaptiveDepth = 5;
		}

		m_agentCfgs[iAgent].steeringBehavior = ca;
	}

	// PathFollowing behavior
	else if (type && type->IsString() && type->AsString() == L"pathFollowing")
	{
		dtPathFollowing* pf = dtPathFollowing::allocate(m_agentCount);
		if (!pf->init(*crowd->getCrowdQuery(), m_navMesh, crowd->getAgents(), crowd->getNbMaxAgents()))
			return;

		m_agentCfgs[iAgent].steeringBehavior = pf;

		dtPathFollowingParams* params = new dtPathFollowingParams;

		params = pf->addBehaviorParams(*crowd->getAgent(iAgent));

		if (params)
		{
			params->debugInfos = 0;
			params->debugIndex = iAgent;
		}

		dtPolyRef dest;
		float nearest[3];
		crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(m_agentCfgs[iAgent].position, crowd->getCrowdQuery()->getQueryExtents(), 
			crowd->getCrowdQuery()->getQueryFilter(), &dest, nearest);
		params->corridor.reset(dest, nearest);

		JSONValue* pathOpti = behavior->Child(L"pathOptimizationRange");
		if (pathOpti && pathOpti->IsNumber())
			params->pathOptimizationRange = (float)pathOpti->AsNumber();
		
		JSONValue* destination = behavior->Child(L"destination");
		if (destination && destination->IsArray())
		{
			m_agentCfgs[iAgent].destination[0] = (float)destination->Child((size_t)0)->AsNumber();
			m_agentCfgs[iAgent].destination[1] = (float)destination->Child(1)->AsNumber();
			m_agentCfgs[iAgent].destination[2] = (float)destination->Child(2)->AsNumber();

			crowd->getCrowdQuery()->getNavMeshQuery()->findNearestPoly(m_agentCfgs[iAgent].destination, crowd->getCrowdQuery()->getQueryExtents(), 
				crowd->getCrowdQuery()->getQueryFilter(), 
				&m_agentCfgs[iAgent].destinationPoly, 0);

			pf->requestMoveTarget(iAgent, m_agentCfgs[iAgent].destinationPoly, m_agentCfgs[iAgent].destination);
		}
	}

	// Seek behavior
	else if (type && type->IsString() && type->AsString() == L"seek")
	{
		dtSeekBehavior* seekBehavior = dtSeekBehavior::allocate(m_agentCount);
		dtSeekBehaviorParams* params;

		params = seekBehavior->addBehaviorParams(*crowd->getAgent(iAgent));

		if (!params)
			return;
		
		m_agentCfgs[iAgent].steeringBehavior = seekBehavior;

		JSONValue* target = behavior->Child(L"targetIdx");
		if (target && target->IsNumber())
			params->seekTarget = &crowd->getAgents()[(int) target->AsNumber()];

		JSONValue* dist = behavior->Child(L"minimalDistance");
		if (dist && dist->IsNumber())
			params->seekDistance = (float) dist->AsNumber();

		JSONValue* predict = behavior->Child(L"predictionFactor");
		if (predict && predict->IsNumber())
			params->seekPredictionFactor = (float) predict->AsNumber();
	}

	// Separation behavior
	else if (type && type->IsString() && type->AsString() == L"separation")
	{		
		dtSeparationBehavior* separationBehavior = dtSeparationBehavior::allocate(m_agentCount);
		dtSeparationBehaviorParams* params;

		params = separationBehavior->addBehaviorParams(*crowd->getAgent(iAgent));

		if (!params)
			return;

		params->separationAgents = crowd->getAgents();

		JSONValue* weight = behavior->Child(L"weight");
		if (weight && weight->IsNumber())
			params->separationWeight = (float) weight->AsNumber();

		JSONValue* dist = behavior->Child(L"distance");
		if (dist && dist->IsNumber())
			params->separationDistance = (float) weight->AsNumber();

		JSONValue* targets = behavior->Child(L"targets");
		if (targets && targets->IsArray())
		{
			for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
			{
				JSONValue* index = targets->Child(i);

				if (index && index->IsNumber())
					m_separationTargets.push_back(static_cast<int>(index->AsNumber()));
			}

			int* targets = (int*) dtAlloc(sizeof(int) * m_separationTargets.size(), DT_ALLOC_PERM);
			std::copy(m_separationTargets.begin(), m_separationTargets.end(), targets);

			params->separationTargets = targets;
			params->separationNbTargets = m_separationTargets.size();
		}

		m_agentCfgs[iAgent].steeringBehavior = separationBehavior;
	}

	// Alignment behavior
	else if (type && type->IsString() && type->AsString() == L"alignment")
	{				
		dtAlignmentBehavior* alignBehavior = dtAlignmentBehavior::allocate(m_agentCount);
		dtAlignmentBehaviorParams* params = new dtAlignmentBehaviorParams;

		params = alignBehavior->addBehaviorParams(*crowd->getAgent(iAgent));

		if (!params)
			return;

		params->alignmentAgents = crowd->getAgents();

		JSONValue* targets = behavior->Child(L"targets");
		if (targets && targets->IsArray())
		{
			for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
			{
				JSONValue* index = targets->Child(i);

				if (index && index->IsNumber())
					m_alignmentTargets.push_back(static_cast<int>(index->AsNumber()));
			}

			int* targets = (int*) dtAlloc(sizeof(int) * m_alignmentTargets.size(), DT_ALLOC_PERM);
			std::copy(m_alignmentTargets.begin(), m_alignmentTargets.end(), targets);

			params->alignmentTargets = targets;
			params->alignmentNbTargets = m_alignmentTargets.size();
		}

		m_agentCfgs[iAgent].steeringBehavior = alignBehavior;
	}

	// Cohesion behavior
	else if (type && type->IsString() && type->AsString() == L"cohesion")
	{				
		dtCohesionBehavior* cohesion = dtCohesionBehavior::allocate(m_agentCount);
		dtCohesionAgentsParams* params;

		params = cohesion->addBehaviorParams(*crowd->getAgent(iAgent));

		if (!params)
			return;

		params->cohesionAgents = crowd->getAgents();

		JSONValue* targets = behavior->Child(L"targets");
		if (targets && targets->IsArray())
		{
			for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
			{
				JSONValue* index = targets->Child(i);

				if (index && index->IsNumber())
					m_cohesionTargets.push_back(static_cast<int>(index->AsNumber()));
			}

			int* targets = (int*) dtAlloc(sizeof(int) * m_cohesionTargets.size(), DT_ALLOC_PERM);
			std::copy(m_cohesionTargets.begin(), m_cohesionTargets.end(), targets);

			params->cohesionTargets = targets;
			params->cohesionNbTargets = m_cohesionTargets.size();
		}

		m_agentCfgs[iAgent].steeringBehavior = cohesion;
	}

	// Flocking behavior
	else if (type && type->IsString() && type->AsString() == L"flocking")
	{				
		dtFlockingBehavior* fb = dtFlockingBehavior::allocate(m_agentCount, 1.f, 1.f, 1.f);
		dtFlockingBehaviorParams* params;

		params = fb->addBehaviorParams(*crowd->getAgent(iAgent));

		if (!params)
			return;

		if (m_flockingsGroups.empty() == false)
		{
			fb->alignmentWeight = m_flockingsGroups.at(0).alignmentWeight;
			fb->cohesionWeight = m_flockingsGroups.at(0).cohesionWeight;
			fb->separationWeight = m_flockingsGroups.at(0).separationWeight;
			params->separationDistance = m_flockingsGroups.at(0).desiredSeparation;
		}

		params->agents = crowd->getAgents();

		JSONValue* targets = behavior->Child(L"targets");
		if (targets && targets->IsArray())
		{
			for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
			{
				JSONValue* index = targets->Child(i);

				if (index && index->IsNumber())
					m_agentsFlockingNeighbors[iAgent].push_back(static_cast<int>(index->AsNumber()));

				int* targets = (int*) dtAlloc(sizeof(int) * m_agentsFlockingNeighbors[iAgent].size(), DT_ALLOC_PERM);
				std::copy(m_agentsFlockingNeighbors[iAgent].begin(), m_agentsFlockingNeighbors[iAgent].end(), targets);

				params->toFlockWith = targets;
				params->nbflockingTargets = m_agentsFlockingNeighbors[iAgent].size();
			}
		}

		m_agentCfgs[iAgent].steeringBehavior = fb;
	}

	if (pipeline)
		m_pipeline[iAgent].push_back(m_agentCfgs[iAgent].steeringBehavior);
}


bool CrowdSample::loadFromFile(const char* fileName)
{
    char* buf = 0;
    FILE* fp = fopen(fileName, "rb");
    if (!fp)
        return false;
    fseek(fp, 0, SEEK_END);
    int bufSize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    buf = new char[bufSize+1];
    if (!buf)
    {
        fclose(fp);
        return false;
    }
    fread(buf, bufSize, 1, fp);
    fclose(fp);
    buf[bufSize]=0;
    return loadFromBuffer(buf);
}

bool CrowdSample::initialize(InputGeom* scene, dtCrowd* crowd, dtNavMesh* navMesh)
{
	m_navMesh = navMesh;

	getSceneFile();
	parseAgentsInfo();
    
    if (!initializeScene(scene))
        return false;
    else if (!initializeNavmesh(*scene, navMesh))
        return false;

	crowd->init(m_agentCount, m_maxRadius, navMesh);
	parseCrowd(crowd);

    if (!initializeCrowd(crowd))
        return false;
    else
        return true;
}

void CrowdSample::computeMaximumRadius()
{
    m_maxRadius = 0.f;

    for (int i(0) ; i < m_agentCount ; ++i)
        m_maxRadius = rcMax(m_maxRadius,m_agentCfgs[i].radius);
}

bool CrowdSample::initializeScene(InputGeom* scene, float* vert, unsigned vertCount, int* tris, unsigned triCount)
{
	if (vertCount == 0 || triCount == 0)
		return false;

	return scene->loadMesh(0, vert, vertCount, tris, triCount);
}

bool CrowdSample::initializeScene(InputGeom* scene)
{
    if (strlen(m_sceneFileName) > 0)
    {
        return scene->loadMesh(0, m_sceneFileName);
    }
    else
    {
        return false;
    }
}

bool CrowdSample::initializeNavmesh(const InputGeom& scene, dtNavMesh* navMesh)
{
	if (!m_context)
		return false;

    NavMeshCreator creator;
    creator.initParameters();
    creator.m_context = m_context;
    creator.m_inputVertices = scene.getMesh()->getVerts();
    creator.m_inputVerticesCount = scene.getMesh()->getVertCount();
    creator.m_inputTriangles = scene.getMesh()->getTris();
    creator.m_inputTrianglesCount = scene.getMesh()->getTriCount();
    rcVcopy(creator.m_min, scene.getMeshBoundsMin());
    rcVcopy(creator.m_max, scene.getMeshBoundsMax());
    creator.m_minimumObstacleClearance = m_maxRadius;
    creator.allocIntermediateResults();
    creator.computeNavMesh();
        
    if (creator.m_success)
    {
        return dtStatusSucceed(navMesh->init(creator.m_outputNavMeshBuffer, creator.m_outputNavMeshBufferSize, DT_TILE_FREE_DATA));
    }
    else 
    {
        return false;
    }
}

bool CrowdSample::initializeCrowd(dtCrowd* crowd)
{    
    dtStatus status;

	for (int i(0) ; i < m_agentCount ; ++i)
    {
		// Pipeline behavior
		if (!m_pipeline[i].empty())
		{
			dtPipelineBehavior* pipeline = dtPipelineBehavior::allocate();
			dtBehavior** behaviors = (dtBehavior**) dtAlloc(sizeof(dtBehavior*) * m_pipeline[i].size(), DT_ALLOC_PERM);
			std::copy(m_pipeline[i].begin(), m_pipeline[i].end(), behaviors);

			pipeline->setBehaviors(behaviors, m_pipeline[i].size());

			m_agentCfgs[i].steeringBehavior = pipeline;
		}

		// Adding the agent to the crowd
        m_agentCfgs[i].index = crowd->addAgent(m_agentCfgs[i].position);

		if (m_agentCfgs[i].index == -1)
			continue;

		crowd->getAgent(m_agentCfgs[i].index)->radius = m_agentCfgs[i].radius;
		crowd->getAgent(m_agentCfgs[i].index)->maxSpeed = m_agentCfgs[i].maxSpeed;
		crowd->getAgent(m_agentCfgs[i].index)->maxAcceleration = m_agentCfgs[i].maxAcceleration;
		crowd->getAgent(m_agentCfgs[i].index)->collisionQueryRange = m_agentCfgs[i].collisionQueryRange;
		crowd->getAgent(m_agentCfgs[i].index)->npos[0] = m_agentCfgs[i].position[0];
		crowd->getAgent(m_agentCfgs[i].index)->npos[1] = m_agentCfgs[i].position[1];
		crowd->getAgent(m_agentCfgs[i].index)->npos[2] = m_agentCfgs[i].position[2];
		crowd->getAgent(m_agentCfgs[i].index)->height = m_agentCfgs[i].height;
		crowd->getAgent(m_agentCfgs[i].index)->behavior = m_agentCfgs[i].steeringBehavior;
		crowd->getAgent(m_agentCfgs[i].index)->updateFlags = m_agentCfgs[i].updateFlags;
    }
	
    return true; 
}
