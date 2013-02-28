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

#include "Application.h"

Application::Application()
: m_currentSample()
//, m_context()
{
    //m_currentSample.m_context = &m_context;
}

Application::~Application()
{
    
}

bool Application::run()
{
    // Initialize the scene, the navmesh and the crowd.
    InputGeom scene;
    dtNavMesh navMesh;
    dtCrowd crowd;
    if (!m_currentSample.initialize(&scene, &crowd, &navMesh))
    {
        return false;
    }
    
    //Create the debug info
    //DebugInfo debugInfo;
    //debugInfo.m_crowd = &crowd;
    //
    ////Create the visualization
    //Visualization visu;
    //
    //visu.m_scene = &scene;
    //visu.m_crowd = &crowd;
    //visu.m_navmesh = &navMesh;
    //visu.m_debugInfo = &debugInfo;
    //
    ////Run the simulation
    //if (!visu.initialize())
    //{
    //    return false;
    //}
    //
    //while (!visu.m_stopRequested)
    //{
    //    if (!visu.update())
    //    {
    //        return false;
    //    }
    //}
    //
    //if (!visu.terminate())
    //{
    //    return false;
    //}
    //
    //return false;
	return true;
}
