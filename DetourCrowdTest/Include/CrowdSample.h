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

#ifndef CROWDSAMPLE_H
#define CROWDSAMPLE_H

//#include "Visualization.h"

#include <DetourCrowd.h>

class rcContext;
class InputGeom;

struct AgentConfiguration
{
    int index;
    float position[3];
    float destination[3];
    dtPolyRef destinationPoly;
    dtCrowdAgentParams parameters;
};

class CrowdSample
{
public:
    CrowdSample();
    ~CrowdSample();
    
    bool loadFromBuffer(const char* data);
    bool loadFromFile(const char* fileName);
	
    bool initialize(InputGeom* scene, dtCrowd* crowd, dtNavMesh* navMesh);
    bool initializeScene(InputGeom* scene, float* vert, unsigned vertCount, int* tris, unsigned triCount);
    bool initializeScene(InputGeom* scene);
    bool initializeNavmesh(const InputGeom& scene, dtNavMesh* navMesh);
    bool initializeCrowd(dtNavMesh& navmesh, dtCrowd* crowd);
	
    void computeMaximumRadius();
    
    char m_sceneFileName[260];
    AgentConfiguration m_agentCfgs[100];
    int m_agentCount;
    rcContext* m_context;
    
private:    
    float m_maxRadius;
};

#endif