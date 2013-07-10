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


#include "StaticConfiguration.h"

#include <DetourCrowd.h>
#include <DetourFlockingBehavior.h>

#include <map>
#include <vector>

#include <map>

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

	/// Method to call when we want to create a scene without using a JSON file.
	/// The vertices and triangles of the scene are passed directly.
	bool initializeScene(InputGeom* scene, float* vert, unsigned vertCount, int* tris, unsigned triCount);

	bool initializeScene(InputGeom* scene);
	bool initializeCrowd(dtNavMesh& navmesh, dtCrowd* crowd);
	bool initializeNavmesh(const InputGeom& scene, dtNavMesh* navMesh);
    
    AgentConfiguration m_agentCfgs[maxAgentCount];
    int m_agentCount;
	rcContext* m_context;
	char m_sceneFileName[maxPathLen];
	float m_maxRadius;
    
private:
	struct Flocking
	{
		size_t nbMaxAgents;
		float distance;
		float desiredSeparation;
		float separationWeight;
		float cohesionWeight;
		float alignmentWeight;
	};

    void computeMaximumRadius();
    
	std::vector<Flocking> m_flockingsGroups;
	std::vector<dtFlockingBehavior*> m_flockingBehaviors;
	std::map<int, std::vector<int> > m_agentsFlockingNeighbors;
	std::map<int, int> m_seekTargets;
	float m_seekDist;
	float m_seekPredict;
	std::vector<int> m_separationTargets;
	float m_separationWeight;
	std::vector<int> m_alignmentTargets;
	std::vector<int> m_cohesionTargets;
};

#endif
