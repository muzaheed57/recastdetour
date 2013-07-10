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
#include "DetourFlockingBehavior.h"
#include "DetourSeekBehavior.h"
#include "DetourSeparationBehavior.h"

#include <Recast.h>

#include <cstdio>
#include <cstring>

CrowdSample::CrowdSample()
: m_agentCfgs()
, m_agentCount(0)
, m_context(0)
, m_maxRadius(0.f)
, m_sceneFileName()
, m_seekDist(0.f)
, m_seekPredict(0.f)
, m_separationWeight(0.f)
{
    strcpy(m_sceneFileName,"");
}

CrowdSample::~CrowdSample()
{
}

bool CrowdSample::loadFromBuffer( const char* data )
{
    JSONValue* root = JSON::Parse(data);

    if (!root)
    {
        //Unable to parse the JSON data.
        return false;
    }
    else
    {
        JSONValue* scene = root->Child(L"scene");
        if (scene && scene->IsObject())
        {
            JSONValue* file = scene->Child(L"file");
            if (file && file->IsString())
            {
                wcstombs(m_sceneFileName,file->AsString().c_str(), maxPathLen);
            }
        }

		JSONValue* flocking = root->Child(L"flockings");

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
					{
						m_flockingsGroups.at(i).desiredSeparation = (float) desiredSeparation->AsNumber();
					}

					JSONValue* separationWeight = flock->Child(L"separationWeight");
					if (separationWeight && separationWeight->IsNumber())
					{
						m_flockingsGroups.at(i).separationWeight = (float) separationWeight->AsNumber();
					}

					JSONValue* cohesionWeight = flock->Child(L"cohesionWeight");
					if (cohesionWeight && cohesionWeight->IsNumber())
					{
						m_flockingsGroups.at(i).cohesionWeight = (float) cohesionWeight->AsNumber();
					}

					JSONValue* alignmentWeight = flock->Child(L"alignmentWeight");
					if (alignmentWeight && alignmentWeight->IsNumber())
					{
						m_flockingsGroups.at(i).alignmentWeight = (float) alignmentWeight->AsNumber();
					}
				}
			}
		}
		
        JSONValue* agents = root->Child(L"agents");
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

                    JSONValue* destination = agent->Child(L"destination");
                    if (destination && destination->IsArray())
                    {
                        m_agentCfgs[iAgent].destination[0] = (float)destination->Child((size_t)0)->AsNumber();
                        m_agentCfgs[iAgent].destination[1] = (float)destination->Child(1)->AsNumber();
                        m_agentCfgs[iAgent].destination[2] = (float)destination->Child(2)->AsNumber();
                    }

                    JSONValue* parameters = agent->Child(L"parameters");
                    if (parameters && parameters->IsObject())
                    {
                        JSONValue* maxSpeed = parameters->Child(L"maxSpeed");
                        if (maxSpeed && maxSpeed->IsNumber())
                        {
                            m_agentCfgs[iAgent].parameters.maxSpeed = (float)maxSpeed->AsNumber();
                        }

                        JSONValue* maxAcceleration = parameters->Child(L"maxAcceleration");
                        if (maxAcceleration && maxAcceleration->IsNumber())
                        {
                            m_agentCfgs[iAgent].parameters.maxAcceleration = (float)maxAcceleration->AsNumber();
                        }

                        JSONValue* radius = parameters->Child(L"radius");
                        if (radius && radius->IsNumber())
                        {
                            m_agentCfgs[iAgent].parameters.radius = (float)radius->AsNumber();
                        }

                        JSONValue* height = parameters->Child(L"height");
                        if (height && height->IsNumber())
                        {
                            m_agentCfgs[iAgent].parameters.height = (float)height->AsNumber();
                        }

                        JSONValue* collisionQueryRange = parameters->Child(L"collisionQueryRange");
                        if (collisionQueryRange && collisionQueryRange->IsNumber())
                        {
                            m_agentCfgs[iAgent].parameters.collisionQueryRange = (float)collisionQueryRange->AsNumber();
                        }

						JSONValue* pathOptimizationRange = parameters->Child(L"pathOptimizationRange");
						if (pathOptimizationRange && pathOptimizationRange->IsNumber())
						{
							m_agentCfgs[iAgent].parameters.pathOptimizationRange = (float)pathOptimizationRange->AsNumber();
						}

						JSONValue* flockingNeighbors = parameters->Child(L"flockingNeighbors");
						if (flockingNeighbors && flockingNeighbors->IsArray())
						{
							for (size_t j = 0; j < flockingNeighbors->CountChildren(); ++j)
								if (flockingNeighbors->Child(j)->IsNumber())
									m_agentsFlockingNeighbors[iAgent].push_back((int) flockingNeighbors->Child(j)->AsNumber());
						}

						JSONValue* behavior = parameters->Child(L"behavior");
						if (behavior && behavior->IsObject())
						{
							JSONValue* type = behavior->Child(L"type");

							// Seek behavior
							if (type && type->IsString() && type->AsString() == L"seek")
							{
								m_agentCfgs[iAgent].parameters.seekBehavior = new dtSeekBehavior;

								JSONValue* target = behavior->Child(L"targetIdx");
								if (target && target->IsNumber())
									m_seekTargets[iAgent] = (int) target->AsNumber();

								JSONValue* dist = behavior->Child(L"minimalDistance");
								if (dist && dist->IsNumber())
									m_seekDist = (float) dist->AsNumber();

								JSONValue* predict = behavior->Child(L"predictionFactor");
								if (predict && predict->IsNumber())
									m_seekPredict = (float) predict->AsNumber();
							}
							// Separation behavior
							else if (type && type->IsString() && type->AsString() == L"separation")
							{												
								JSONValue* weight = behavior->Child(L"weight");
								if (weight && weight->IsNumber())
									m_separationWeight = (float) weight->AsNumber();

								JSONValue* targets = behavior->Child(L"targets");
								if (targets && targets->IsArray())
								{
									for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
									{
										JSONValue* index = targets->Child(i);
							
										if (index && index->IsNumber())
											m_separationTargets.push_back(index->AsNumber());
									}
								}
							}
							// Alignment behavior
							else if (type && type->IsString() && type->AsString() == L"alignment")
							{				
								m_agentCfgs[iAgent].parameters.alignmentBehavior = new dtAlignmentBehavior;
								JSONValue* targets = behavior->Child(L"targets");
								if (targets && targets->IsArray())
								{
									for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
									{
										JSONValue* index = targets->Child(i);

										if (index && index->IsNumber())
											m_alignmentTargets.push_back(index->AsNumber());
									}
								}
							}
							// Cohesion behavior
							else if (type && type->IsString() && type->AsString() == L"cohesion")
							{				
								m_agentCfgs[iAgent].parameters.cohesionBehavior = new dtCohesionBehavior;
								JSONValue* targets = behavior->Child(L"targets");
								if (targets && targets->IsArray())
								{
									for (std::size_t i(0), size(targets->CountChildren()) ; i < size ; ++i)
									{
										JSONValue* index = targets->Child(i);

										if (index && index->IsNumber())
											m_cohesionTargets.push_back(index->AsNumber());
									}
								}
							}
						}
						
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
                                    {
                                        m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
                                    }
                                    else if (updateFlagStr==L"DT_CROWD_OBSTACLE_AVOIDANCE")
                                    {
                                        m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
                                    }
                                    else if (updateFlagStr==L"DT_CROWD_SEPARATION")
                                    {
                                        m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_SEPARATION;
                                    }
                                    else if (updateFlagStr==L"DT_CROWD_OPTIMIZE_VIS")
                                    {
                                        m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
                                    }
                                    else if (updateFlagStr==L"DT_CROWD_OPTIMIZE_TOPO")
                                    {
                                        m_agentCfgs[iAgent].parameters.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        delete root;
        return true;
    }
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
    computeMaximumRadius();
    
    if (!initializeScene(scene))
    {
        return false;
    }
    else if (!initializeNavmesh(*scene, navMesh))
    {
        return false;
    }
    else if (!initializeCrowd(*navMesh, crowd))
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
    crowd->init(m_agentCount, m_maxRadius, &navmesh);
    
    dtStatus status;

	for (int i(0) ; i < m_agentCount ; ++i)
    {
		// Seeking behavior
		if (m_agentCfgs[i].parameters.seekBehavior)
		{
			m_agentCfgs[i].parameters.seekBehavior->m_distance = m_seekDist;
			m_agentCfgs[i].parameters.seekBehavior->m_predictionFactor = m_seekPredict;

			if (m_seekTargets.find(i) != m_seekTargets.end())
				m_agentCfgs[i].parameters.seekBehavior->setTarget(crowd->getAgent(m_seekTargets[i]));
		}

		// Separation behavior
		if (!m_separationTargets.empty())
		{
			m_agentCfgs[i].parameters.separationBehavior = new dtSeparationBehavior;
			m_agentCfgs[i].parameters.separationBehavior->m_separationWeight = m_separationWeight;
			m_agentCfgs[i].parameters.separationBehavior->setAgents(crowd->getAgents());
			int* targets = (int*) dtAlloc(sizeof(int) * m_separationTargets.size(), DT_ALLOC_PERM);

			std::copy(m_separationTargets.begin(), m_separationTargets.end(), targets);

			m_agentCfgs[i].parameters.separationBehavior->setTargets(targets, m_separationTargets.size());
		}

		// alignment behavior
		if (m_agentCfgs[i].parameters.alignmentBehavior)
		{
			m_agentCfgs[i].parameters.alignmentBehavior->setAgents(crowd->getAgents());

			if (!m_alignmentTargets.empty())
			{
				int* targets = (int*) dtAlloc(sizeof(int) * m_alignmentTargets.size(), DT_ALLOC_PERM);

				std::copy(m_alignmentTargets.begin(), m_alignmentTargets.end(), targets);

				m_agentCfgs[i].parameters.alignmentBehavior->setTargets(targets, m_alignmentTargets.size());
			}
		}

		// cohesion behavior
		if (m_agentCfgs[i].parameters.cohesionBehavior)
		{
			m_agentCfgs[i].parameters.cohesionBehavior->setAgents(crowd->getAgents());

			if (!m_cohesionTargets.empty())
			{
				int* targets = (int*) dtAlloc(sizeof(int) * m_cohesionTargets.size(), DT_ALLOC_PERM);

				std::copy(m_cohesionTargets.begin(), m_cohesionTargets.end(), targets);

				m_agentCfgs[i].parameters.cohesionBehavior->setTargets(targets, m_cohesionTargets.size());
			}
		}

		// Flocking behavior targets
		if (m_agentsFlockingNeighbors[i].empty() == false)
		{
			std::vector<int>& v = m_agentsFlockingNeighbors.at(i);
			int* flockingNeighbList = new int[v.size()];

			for (int j = 0; j < v.size(); ++j)
				flockingNeighbList[j] = v.at(j);

			m_agentCfgs[i].parameters.toFlockWith = flockingNeighbList;
			m_agentCfgs[i].parameters.nbFlockingNeighbors = v.size();
		}

        m_agentCfgs[i].index = crowd->addAgent(m_agentCfgs[i].position, &m_agentCfgs[i].parameters);
        
        status = crowd->getNavMeshQuery()->findNearestPoly(m_agentCfgs[i].destination,crowd->getQueryExtents(),crowd->getFilter(),&m_agentCfgs[i].destinationPoly,0);
        
        if (dtStatusFailed(status))
        {
            return false;
        }
        
        status = crowd->requestMoveTarget(m_agentCfgs[i].index, m_agentCfgs[i].destinationPoly, m_agentCfgs[i].destination);
        
        if (dtStatusFailed(status))
        {
            return false;
        }
    }

	// Flocking behavior
	m_flockingBehaviors.resize(m_flockingsGroups.size());

	for (std::vector<Flocking>::const_iterator it = m_flockingsGroups.begin(); it < m_flockingsGroups.end(); ++it)
	{
		dtFlockingBehavior* fb = new dtFlockingBehavior(it->desiredSeparation, 
														it->separationWeight, it->cohesionWeight, it->alignmentWeight, 
														crowd->getAgents());

		for (int i(0) ; i < m_agentCount ; ++i)
			if (m_agentsFlockingNeighbors[i].empty() == false)
				crowd->getAgent(i)->params.flockingBehavior = fb;

		m_flockingBehaviors.push_back(fb);
	}

    return true; 
}
