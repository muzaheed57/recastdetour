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

#include "DetourSeekBehavior.h"
#include "DetourAlignmentBehavior.h"
#include "DetourCohesionBehavior.h"
#include "DetourCollisionAvoidance.h"
#include "DetourPathFollowing.h"
#include "DetourPipelineBehavior.h"
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
						m_agentCfgs[iAgent].parameters.maxSpeed = (float)maxSpeed->AsNumber();

					JSONValue* maxAcceleration = parameters->Child(L"maxAcceleration");
					if (maxAcceleration && maxAcceleration->IsNumber())
						m_agentCfgs[iAgent].parameters.maxAcceleration = (float)maxAcceleration->AsNumber();

					JSONValue* radius = parameters->Child(L"radius");
					if (radius && radius->IsNumber())
						m_agentCfgs[iAgent].parameters.radius = (float)radius->AsNumber();

					JSONValue* height = parameters->Child(L"height");
					if (height && height->IsNumber())
						m_agentCfgs[iAgent].parameters.height = (float)height->AsNumber();

					JSONValue* collisionQueryRange = parameters->Child(L"collisionQueryRange");
					if (collisionQueryRange && collisionQueryRange->IsNumber())
						m_agentCfgs[iAgent].parameters.collisionQueryRange = (float)collisionQueryRange->AsNumber();

					JSONValue* pathOptimizationRange = parameters->Child(L"pathOptimizationRange");
					if (pathOptimizationRange && pathOptimizationRange->IsNumber())
						m_agentCfgs[iAgent].parameters.pathOptimizationRange = (float)pathOptimizationRange->AsNumber();

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
									m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
								else if (updateFlagStr==L"DT_CROWD_OBSTACLE_AVOIDANCE")
									m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
								else if (updateFlagStr==L"DT_CROWD_SEPARATION")
									m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_SEPARATION;
								else if (updateFlagStr==L"DT_CROWD_OPTIMIZE_VIS")
									m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
								else if (updateFlagStr==L"DT_CROWD_OPTIMIZE_TOPO")
									m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
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
		dtCollisionAvoidance* ca = dtAllocBehavior<dtCollisionAvoidance>();
		ca->init();
		m_agentCfgs[iAgent].parameters.caAgents = crowd->getAgents();
		m_agentCfgs[iAgent].parameters.caDebug = 0;
		m_agentCfgs[iAgent].parameters.steeringBehavior = ca;
	}

	// PathFollowing behavior
	else if (type && type->IsString() && type->AsString() == L"pathFollowing")
	{
		dtPathFollowing* pf = dtAllocBehavior<dtPathFollowing>();
		if (!pf->init(crowd->getPathQueue(), crowd->getNavMeshQuery(), crowd->getQueryExtents(), crowd->getEditableFilter(), crowd->getMaxPathResult(), 
				 crowd->getAgents(), crowd->getNbMaxAgents(), crowd->getAnims()))
			return;

		m_agentCfgs[iAgent].parameters.steeringBehavior = pf;

		m_agentCfgs[iAgent].parameters.pfAnims = crowd->getAnims();
		m_agentCfgs[iAgent].parameters.pfDebug = 0;
		m_agentCfgs[iAgent].parameters.pfDebugIbdex = iAgent;

		JSONValue* destination = behavior->Child(L"destination");
		if (destination && destination->IsArray())
		{
			m_agentCfgs[iAgent].destination[0] = (float)destination->Child((size_t)0)->AsNumber();
			m_agentCfgs[iAgent].destination[1] = (float)destination->Child(1)->AsNumber();
			m_agentCfgs[iAgent].destination[2] = (float)destination->Child(2)->AsNumber();
		}
	}

	// Seek behavior
	else if (type && type->IsString() && type->AsString() == L"seek")
	{
		m_agentCfgs[iAgent].parameters.steeringBehavior = dtAllocBehavior<dtSeekBehavior>();
		m_agentsBehaviors[iAgent] = "seek";

		JSONValue* target = behavior->Child(L"targetIdx");
		if (target && target->IsNumber())
			m_agentCfgs[iAgent].parameters.seekTarget = &crowd->getAgents()[(int) target->AsNumber()];

		JSONValue* dist = behavior->Child(L"minimalDistance");
		if (dist && dist->IsNumber())
			m_agentCfgs[iAgent].parameters.seekDistance = (float) dist->AsNumber();

		JSONValue* predict = behavior->Child(L"predictionFactor");
		if (predict && predict->IsNumber())
			m_agentCfgs[iAgent].parameters.seekPredictionFactor = (float) predict->AsNumber();
	}

	// Separation behavior
	else if (type && type->IsString() && type->AsString() == L"separation")
	{		
		m_agentCfgs[iAgent].parameters.steeringBehavior = dtAllocBehavior<dtSeparationBehavior>();
		m_agentsBehaviors[iAgent] = "separation";

		m_agentCfgs[iAgent].parameters.separationAgents = crowd->getAgents();

		JSONValue* weight = behavior->Child(L"weight");
		if (weight && weight->IsNumber())
			m_agentCfgs[iAgent].parameters.separationWeight = (float) weight->AsNumber();

		JSONValue* dist = behavior->Child(L"distance");
		if (dist && dist->IsNumber())
			m_agentCfgs[iAgent].parameters.separationDistance = (float) weight->AsNumber();

		JSONValue* targets = behavior->Child(L"targets");
		if (targets && targets->IsArray())
		{
			for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
			{
				JSONValue* index = targets->Child(i);

				if (index && index->IsNumber())
					m_separationTargets.push_back(index->AsNumber());
			}

			int* targets = (int*) dtAlloc(sizeof(int) * m_separationTargets.size(), DT_ALLOC_PERM);
			std::copy(m_separationTargets.begin(), m_separationTargets.end(), targets);

			m_agentCfgs[iAgent].parameters.separationTargets = targets;
			m_agentCfgs[iAgent].parameters.separationNbTargets = m_separationTargets.size();
		}
	}

	// Alignment behavior
	else if (type && type->IsString() && type->AsString() == L"alignment")
	{				
		m_agentCfgs[iAgent].parameters.steeringBehavior = dtAllocBehavior<dtAlignmentBehavior>();
		m_agentsBehaviors[iAgent] = "alignment";
		m_agentCfgs[iAgent].parameters.alignmentAgents = crowd->getAgents();

		JSONValue* targets = behavior->Child(L"targets");
		if (targets && targets->IsArray())
		{
			for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
			{
				JSONValue* index = targets->Child(i);

				if (index && index->IsNumber())
					m_alignmentTargets.push_back(index->AsNumber());
			}

			int* targets = (int*) dtAlloc(sizeof(int) * m_alignmentTargets.size(), DT_ALLOC_PERM);
			std::copy(m_alignmentTargets.begin(), m_alignmentTargets.end(), targets);

			m_agentCfgs[iAgent].parameters.alignmentTargets = targets;
			m_agentCfgs[iAgent].parameters.alignmentNbTargets = m_alignmentTargets.size();
		}
	}

	// Cohesion behavior
	else if (type && type->IsString() && type->AsString() == L"cohesion")
	{				
		m_agentCfgs[iAgent].parameters.steeringBehavior = dtAllocBehavior<dtCohesionBehavior>();
		m_agentsBehaviors[iAgent] = "cohesion";
		m_agentCfgs[iAgent].parameters.cohesionAgents = crowd->getAgents();

		JSONValue* targets = behavior->Child(L"targets");
		if (targets && targets->IsArray())
		{
			for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
			{
				JSONValue* index = targets->Child(i);

				if (index && index->IsNumber())
					m_cohesionTargets.push_back(index->AsNumber());
			}

			int* targets = (int*) dtAlloc(sizeof(int) * m_cohesionTargets.size(), DT_ALLOC_PERM);
			std::copy(m_cohesionTargets.begin(), m_cohesionTargets.end(), targets);

			m_agentCfgs[iAgent].parameters.cohesionTargets = targets;
			m_agentCfgs[iAgent].parameters.cohesionNbTargets = m_cohesionTargets.size();
		}
	}

	// Flocking behavior
	else if (type && type->IsString() && type->AsString() == L"flocking")
	{				
		dtFlockingBehavior* fb = dtAllocBehavior<dtFlockingBehavior>();

		if (m_flockingsGroups.empty() == false)
		{
			fb->m_alignmentWeight = m_flockingsGroups.at(0).alignmentWeight;
			fb->m_cohesionWeight = m_flockingsGroups.at(0).cohesionWeight;
			fb->m_separationWeight = m_flockingsGroups.at(0).separationWeight;
			m_agentCfgs[iAgent].parameters.flockingSeparationDistance = m_flockingsGroups.at(0).desiredSeparation;
		}

		m_agentCfgs[iAgent].parameters.steeringBehavior = fb;
		m_agentsBehaviors[iAgent] = "flocking";
		m_agentCfgs[iAgent].parameters.flockingAgents = crowd->getAgents();

		JSONValue* targets = behavior->Child(L"targets");
		if (targets && targets->IsArray())
		{
			for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
			{
				JSONValue* index = targets->Child(i);

				if (index && index->IsNumber())
					m_agentsFlockingNeighbors[iAgent].push_back(index->AsNumber());

				int* targets = (int*) dtAlloc(sizeof(int) * m_agentsFlockingNeighbors[iAgent].size(), DT_ALLOC_PERM);
				std::copy(m_agentsFlockingNeighbors[iAgent].begin(), m_agentsFlockingNeighbors[iAgent].end(), targets);

				m_agentCfgs[iAgent].parameters.toFlockWith = targets;
				m_agentCfgs[iAgent].parameters.nbFlockingNeighbors = m_agentsFlockingNeighbors[iAgent].size();
			}
		}
	}

	if (pipeline)
		m_pipeline[iAgent].push_back(m_agentCfgs[iAgent].parameters.steeringBehavior);
}


bool CrowdSample::loadFromFile( const char* fileName )
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
	getSceneFile();
	parseAgentsInfo();
    
    if (!initializeScene(scene))
    {
        return false;
    }
    else if (!initializeNavmesh(*scene, navMesh))
    {
        return false;
    }

	crowd->init(m_agentCount, m_maxRadius, navMesh);
	parseCrowd(crowd);

    if (!initializeCrowd(*navMesh, crowd))
    {
        return false;
    }
    else
    {
        return true;
    }
}

void CrowdSample::computeMaximumRadius()
{
    m_maxRadius = 0.f;
    for (int i(0) ; i < m_agentCount ; ++i)
    {
        m_maxRadius = rcMax(m_maxRadius,m_agentCfgs[i].parameters.radius);
    }
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

bool CrowdSample::initializeCrowd(dtNavMesh& navmesh, dtCrowd* crowd)
{    
    dtStatus status;

	for (int i(0) ; i < m_agentCount ; ++i)
    {
		// Pipeline behavior
		if (!m_pipeline[i].empty())
		{
			dtPipelineBehavior* pipeline = dtAllocBehavior<dtPipelineBehavior>();
			dtBehavior** behaviors = (dtBehavior**) dtAlloc(sizeof(dtBehavior*) * m_pipeline[i].size(), DT_ALLOC_PERM);
			std::copy(m_pipeline[i].begin(), m_pipeline[i].end(), behaviors);

			pipeline->setBehaviors(behaviors, m_pipeline[i].size());

			m_agentCfgs[i].parameters.steeringBehavior = pipeline;
		}

        m_agentCfgs[i].index = crowd->addAgent(m_agentCfgs[i].position, &m_agentCfgs[i].parameters);
        
        status = crowd->getNavMeshQuery()->findNearestPoly(m_agentCfgs[i].destination,crowd->getQueryExtents(),crowd->getFilter(),&m_agentCfgs[i].destinationPoly,0);
        
        if (dtStatusFailed(status))
            return false;
        
        status = crowd->requestMoveTarget(m_agentCfgs[i].index, m_agentCfgs[i].destinationPoly, m_agentCfgs[i].destination);
        
        if (dtStatusFailed(status))
            return false;
    }
	
    return true; 
}
