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

#include "DebugInfo.h"
#include "DetourCollisionAvoidance.h"

#include <DetourCommon.h>

#include <cstring>

DebugInfo::DebugInfo()
: m_crowd(0)
, m_crowdTotalTime()
, m_crowdSampleCount()
, m_initialized(false)
, m_lastStart()
{	
    memset(m_agentTrails, 0, sizeof(m_agentTrails));
}

DebugInfo::~DebugInfo()
{
}

bool DebugInfo::initialize()
{
    m_initialized = m_crowd != 0;
    
    // Init trails
    for (int i = 0, size(m_crowd->getAgentCount()); m_initialized && i < size; ++i)
    {
        const dtCrowdAgent* ag = m_crowd->getAgent(i);
        if (ag->active)
        {
            AgentTrail* trail = &m_agentTrails[i];
            for (int j(0); j < maxAgentTrailLen; ++j)
                dtVcopy(&trail->trail[j*3], ag->position);
            trail->htrail = 0;
        }
    }
    
    return m_initialized;
}

bool DebugInfo::terminate()
{
    m_initialized = false;
    return !m_initialized;
}

bool DebugInfo::startUpdate()
{
    m_lastStart = getPerfTime();
    return true;
}

bool DebugInfo::endUpdate(float dt)
{
    TimeVal end = getPerfTime();
    
    //m_crowdSampleCount.addSample((float)m_crowd->getVelocitySampleCount());
    m_crowdTotalTime.addSample(getPerfDeltaTimeUsec(m_lastStart, end) / 1000.0f);
    
    // Update agent trails
    for (int i = 0, size(m_crowd->getAgentCount()); i < size; ++i)
    {
        const dtCrowdAgent* ag = m_crowd->getAgent(i);
        if (ag->active)
        {
            AgentTrail* trail = &m_agentTrails[i];
            
            // Update agent movement trail.
            trail->htrail = (trail->htrail + 1) % maxAgentTrailLen;
            dtVcopy(&trail->trail[trail->htrail*3], ag->position);
        }
    }
    
    return true;
}

bool DebugInfo::isInitialized() const
{
    return m_initialized;
}