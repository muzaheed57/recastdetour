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

dtCrowd* TestScene::createSquareScene()
{
	// Creation of a square
	float* vert = new float[12];
	int* tri = new int[6];

	vert[0] = 20.0; vert[1] = 0.0; vert[2] = 20.0;
	vert[3] = 20.0; vert[4] = 0.0; vert[5] = -20.0;
	vert[6] = -20.0; vert[7] = 0.0; vert[8] = -20.0;
	vert[9] = -20.0; vert[10] = 0.0; vert[11] = 20.0;

	tri[0] = 0; tri[1] = 1; tri[2] = 2;
	tri[3] = 2; tri[4] = 3; tri[5] = 0;

	if (!m_cs.initializeScene(&m_scene, vert, 4, tri, 2)) 
		return 0;

	if (!m_cs.initializeNavmesh(m_scene, &m_navMesh)) 
		return 0;

	if (!m_crowd->init(20, 0.5, &m_navMesh)) 
		return 0;

	return m_crowd;
}

bool TestScene::defaultInitializeAgent(dtCrowd& crowd, int index) const
{
	if (index == -1)
		return false;

	dtCrowdAgent* ag = crowd.getAgent(index);

	ag->radius = 0.2;
	ag->height = 1.7;
	ag->maxSpeed = 2.0;
	ag->maxAcceleration = 10.0;
	ag->collisionQueryRange = 4.0;
	ag->updateFlags = DT_CROWD_OBSTACLE_AVOIDANCE;
	ag->behavior = 0;

	return true;
}
