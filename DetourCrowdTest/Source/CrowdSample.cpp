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

#include <Recast.h>

#include <cstdio>
#include <cstring>

CrowdSample::CrowdSample()
: m_sceneFileName()
, m_agentCfgs()
, m_agentCount(0)
, m_context(0)
, m_maxRadius(0.f)
{
    strcpy(m_sceneFileName, "");
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
                wcstombs(m_sceneFileName,file->AsString().c_str(), 260);
            }
        }

        JSONValue* agents = root->Child(L"agents");
        if (agents && agents->IsArray())
        {
            m_agentCount = rcMin<int>(agents->CountChildren(),100);
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
    return true; 
}
