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

#include "DetourCrowdTestUtils.h"

TestScene::TestScene()
	: m_crowd(0)
{
	m_crowd = new dtCrowd;
	m_cs.m_maxRadius = 0.2f;
	m_cs.m_context = &m_bc;
}

TestScene::~TestScene()
{
	delete m_crowd;
	m_crowd = 0;
}

dtCrowd* TestScene::createScene(dtCrowdAgentParams& param, float* vertices, int* triangles)
{
	if (!m_cs.initializeScene(&m_scene, vertices, 4, triangles, 2)) 
		return 0;

	if (!m_cs.initializeNavmesh(m_scene, &m_navMesh)) 
		return 0;

	if (!m_crowd->init(20, 0.5, &m_navMesh)) 
		return 0;

	memset(&param, 0, sizeof(dtCrowdAgentParams));
	param.radius = 0.2;
	param.height = 1.7;
	param.maxSpeed = 2.0;
	param.maxAcceleration = 10.0;
	param.collisionQueryRange = 4.0;
	param.pathOptimizationRange = 6.0;
	param.updateFlags = DT_CROWD_OBSTACLE_AVOIDANCE;
	param.obstacleAvoidanceType = 0;
	param.steeringBehavior = 0;

	return m_crowd;
}
