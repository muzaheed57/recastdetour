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

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "ValueHistory.h"
#include "StaticConfiguration.h"

#include <DetourCrowd.h>

#include <SDL_opengl.h>

#include <stdint.h>

class InputGeom;
class DebugInfo;

class Visualization
{
public:
    Visualization();
    ~Visualization();
    
    bool initialize();
    bool terminate();
    bool update();

    bool isInitialized() const;
    
    void setFullscreen(bool fullscreen);
    bool isFullscreen() const;
    
    void setDebuggedAgent(int agentIdx);
    int getDebuggedAgent();
    
    bool pick(float* position, int* agentIdx);
    
    float m_zNear;
    float m_zFar;
    float m_cameraPosition[3]; //!< Camera position (x, y, z)
    float m_cameraOrientation[3]; //!< Camera rotation (euler angles, in degrees, around x, y ,z)
    float m_cameraVelocity[3]; //!< Camera's velocity (x, y, z)
    InputGeom* m_scene;
    dtNavMesh* m_navmesh;
    dtCrowd* m_crowd;
    DebugInfo* m_debugInfo;
    
    float m_crowdDt; //!< The dt to which the crowd will be updated 
    
    bool m_stopRequested;
    
private:
    
    bool initializeCamera();
    
    void updateCameraVelocity(float dt, bool forward, bool backward, bool left, bool right, bool fast);
    void updateCameraPosition(float dt); 
    
    void renderScene();
    void renderCrowd();
    
    void renderDebugInfoOverlay();
    
    bool m_fullscreen;
    bool m_initialized;
    bool m_paused;
    
    int m_winHeight;
    int m_winWidth;
    uint32_t m_lastTickCount;
    int m_mousePosition[2];
    bool m_rotating;
    float m_intialCameraOrientation[3];
    int m_initialMousePosition[2];
    
    GLdouble m_projection[16];
    GLdouble m_modelView[16];
    GLint m_viewport[4];
    
    float m_crowdAvailableDt;
};

#endif