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

#ifndef DETOURCROWDTESTUTILS_H
#define DETOURCROWDTESTUTILS_H

#include "DetourCrowd.h"

#include "CrowdSample.h"
#include "InputGeom.h"
#include "DetourSceneCreator.h"

/// Class used to create a scene from vertices and triangles.
class TestScene
{
public:
	TestScene();
	~TestScene();

	/// Creation of the 3D scene (a square).
	///
	/// @param[in]	nbMaxAgents	The maximum number of agents allowed for the crowd
	/// @param[in]	maxRadius	The radius allowed for the agents of the crowd
	///
	/// @return Return the newly created crowd. Return 0 if something went wrong.
	dtCrowd* createSquareScene(unsigned nbMaxAgents, float maxRadius);

	bool defaultInitializeAgent(dtCrowd& crowd, int index) const;

	dtNavMesh* getNavMesh() { return &m_navMesh; }

private:
	CrowdSample m_cs;
	InputGeom m_scene;
	dtNavMesh m_navMesh;
	BuildContext m_bc;
	dtCrowd* m_crowd;
};

#endif