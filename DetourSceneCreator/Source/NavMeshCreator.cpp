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

#include "NavMeshCreator.h"

#include <DetourNavMeshBuilder.h>
#include <DetourNavMesh.h>

#include <Recast.h>
#include <RecastAlloc.h>
#include <RecastDump.h>

#include <cmath>
#include <cstring>


void NavMeshCreator::initParameters()
{
    m_voxelSize = 0.1f;
    m_voxelHeight = 0.2f;
    m_minimumCeilingClearance = 2.0f;
    m_maximumStepHeight = 0.3f;
    m_minimumObstacleClearance = 0.3f;
    m_maximumSlope = 45.0f;
    m_edgeMaxError = 1.3f;
    m_edgeMaxLength = 12.f;
    m_polyMaxNbVertices = 6;
    m_sampleDist = 0.1f;
    m_sampleMaxError = 0.2f;
    m_regionMinSize = 8;
    m_regionMergeSize = 20;
    
    m_success = true;
}

void NavMeshCreator::computeNavMesh()
{
    // Reset build times gathering.
	m_context->resetTimers();
    
    // Start the build process.	
	m_context->startTimer(RC_TIMER_TOTAL);
    m_context->log(RC_LOG_PROGRESS, "NavMesh computation start");
    m_context->log(RC_LOG_PROGRESS, " - %.1fK vertices, %.1fK triangles", m_inputVerticesCount/1000.0f, m_inputTrianglesCount/1000.0f);
    
    if (m_success)
    {
        //Compute the grid size
        rcCalcGridSize(
                       m_min, 
                       m_max, 
                       m_voxelSize, 
                       &m_intermediateHeightfieldWidth, 
                       &m_intermediateHeightfieldHeight);
    }
    
    m_context->log(RC_LOG_PROGRESS, " - %d x %d = %d voxels", m_intermediateHeightfieldWidth, m_intermediateHeightfieldHeight, m_intermediateHeightfieldHeight * m_intermediateHeightfieldWidth);
    
    //Mark the walkable m_inputTriangles
    rcMarkWalkableTriangles(
                            m_context, 
                            m_maximumSlope, 
                            m_inputVertices, 
                            m_inputVerticesCount, 
                            m_inputTriangles, 
                            m_inputTrianglesCount, 
                            m_intermediateTriangleTags);
    
    
    
    // Build the heightfield
    m_success = m_success && (rcCreateHeightfield(
                                              m_context,
                                              *m_intermediateHeightfield,
                                              m_intermediateHeightfieldWidth,
                                              m_intermediateHeightfieldHeight,
                                              m_min,
                                              m_max,
                                              m_voxelSize,
                                              m_voxelHeight));
    
    // rasterize m_inputTriangles.
    rcRasterizeTriangles(
                         m_context,
                         m_inputVertices,
                         m_inputVerticesCount,
                         m_inputTriangles,
                         m_intermediateTriangleTags,
                         m_inputTrianglesCount,
                         *m_intermediateHeightfield,
                         static_cast<int>(floor(m_maximumStepHeight / m_voxelHeight)));
    
    // Filter voxels
    rcFilterLowHangingWalkableObstacles(
                                        m_context, 
                                        static_cast<int>(floor(m_maximumStepHeight / m_voxelHeight)), 
                                        *m_intermediateHeightfield);
    rcFilterLedgeSpans(
                       m_context, 
                       static_cast<int>(ceil(m_minimumCeilingClearance / m_voxelHeight)), 
                       static_cast<int>(floor(m_maximumStepHeight / m_voxelHeight)), 
                       *m_intermediateHeightfield);
    rcFilterWalkableLowHeightSpans(
                                   m_context, 
                                   static_cast<int>(ceil(m_minimumCeilingClearance / m_voxelHeight)), 
                                   *m_intermediateHeightfield);
    
    // Build the compact representation for the heightfield
    m_success = m_success && (rcBuildCompactHeightfield(
                                                    m_context,
                                                    static_cast<int>(ceil(m_minimumCeilingClearance / m_voxelHeight)),
                                                    static_cast<int>(floor(m_maximumStepHeight / m_voxelHeight)),
                                                    *m_intermediateHeightfield,
                                                    *m_intermediateCompactHeightfield));
    
    // Erode the navigatble area by minimum clearance to obstacles
    m_success = m_success && (rcErodeWalkableArea(
                                              m_context, 
                                              static_cast<int>(ceil(m_minimumObstacleClearance / m_voxelSize)), 
                                              *m_intermediateCompactHeightfield));
    
    // Prepare for region partitioning, by calculating distance field along the walkable surface.
    m_success = m_success && (rcBuildDistanceField(
                                               m_context, 
                                               *m_intermediateCompactHeightfield));
    
    // Partition the walkable surface into simple regions without holes.
    m_success = m_success && (rcBuildRegions(
                                         m_context, 
                                         *m_intermediateCompactHeightfield, 
                                         0, 
                                         rcSqr<int>(m_regionMinSize), 
                                         rcSqr<int>(m_regionMergeSize)));
    
    // Build the contours of the walkable surface
    m_success = m_success && (rcBuildContours(
                                          m_context,
                                          *m_intermediateCompactHeightfield,
                                          m_edgeMaxError,
                                          static_cast<int>(ceil(m_edgeMaxLength / m_voxelSize)),
                                          *m_intermediateContourSet));
    
    // Build the polygon mesh, i.e. the navigation mesh geometry
    m_success = m_success && (rcBuildPolyMesh(
                                          m_context,
                                          *m_intermediateContourSet,
                                          m_polyMaxNbVertices,
                                          *m_intermediatePolyMesh));
    
    //Build the detailed polygon mesh, i.e. the detail for the ground.
    m_success = m_success && (rcBuildPolyMeshDetail(
                                                m_context,
                                                *m_intermediatePolyMesh,
                                                *m_intermediateCompactHeightfield,
                                                m_sampleDist,
                                                m_sampleMaxError,
                                                *m_intermediatePolyMeshDetail));
    
    // Update poly flags from areas.
    for (int i = 0; m_success && i < m_intermediatePolyMesh->npolys; ++i)
    {
        switch(m_intermediatePolyMesh->areas[i])
        {
            case RC_WALKABLE_AREA:
                m_intermediatePolyMesh->areas[i] = area::Ground;
                break;
            case RC_NULL_AREA:
            default:
                m_intermediatePolyMesh->areas[i] = area::Obstacle;
                break;
        }
        switch(m_intermediatePolyMesh->areas[i])
        {
            case area::Ground:
                m_intermediatePolyMesh->flags[i] = navigationFlags::Walkable;
                break;
            case area::Obstacle:
            default:
                m_intermediatePolyMesh->flags[i] = navigationFlags::NonWalkable;
                break;
        }
    }
    
    if (m_polyMaxNbVertices > DT_VERTS_PER_POLYGON)
    {
        m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to create Detour NavMesh, the configured maximum number of vertex per polygon (%d) is over Detour's limit (%d).",m_polyMaxNbVertices,DT_VERTS_PER_POLYGON);
        m_success = false;
    }
    if (m_success)
    {
        memset(m_intermediateNavMeshCreateParams, 0, sizeof(dtNavMeshCreateParams));
        m_intermediateNavMeshCreateParams->verts = m_intermediatePolyMesh->verts;
        m_intermediateNavMeshCreateParams->vertCount = m_intermediatePolyMesh->nverts;
        m_intermediateNavMeshCreateParams->polys = m_intermediatePolyMesh->polys;
        m_intermediateNavMeshCreateParams->polyAreas = m_intermediatePolyMesh->areas;
        m_intermediateNavMeshCreateParams->polyFlags = m_intermediatePolyMesh->flags;
        m_intermediateNavMeshCreateParams->polyCount = m_intermediatePolyMesh->npolys;
        m_intermediateNavMeshCreateParams->nvp = m_intermediatePolyMesh->nvp;
        m_intermediateNavMeshCreateParams->detailMeshes = m_intermediatePolyMeshDetail->meshes;
        m_intermediateNavMeshCreateParams->detailVerts = m_intermediatePolyMeshDetail->verts;
        m_intermediateNavMeshCreateParams->detailVertsCount = m_intermediatePolyMeshDetail->nverts;
        m_intermediateNavMeshCreateParams->detailTris = m_intermediatePolyMeshDetail->tris;
        m_intermediateNavMeshCreateParams->detailTriCount = m_intermediatePolyMeshDetail->ntris;

        m_intermediateNavMeshCreateParams->offMeshConVerts = m_offMeshConnectionCreator.vert;
        m_intermediateNavMeshCreateParams->offMeshConRad = m_offMeshConnectionCreator.radius;
        m_intermediateNavMeshCreateParams->offMeshConDir = m_offMeshConnectionCreator.bidir;
        m_intermediateNavMeshCreateParams->offMeshConAreas = m_offMeshConnectionCreator.areas;
        m_intermediateNavMeshCreateParams->offMeshConFlags = m_offMeshConnectionCreator.flags;
        m_intermediateNavMeshCreateParams->offMeshConUserID = m_offMeshConnectionCreator.ids;
        m_intermediateNavMeshCreateParams->offMeshConCount = m_offMeshConnectionCreator.count;

        m_intermediateNavMeshCreateParams->walkableHeight = m_minimumCeilingClearance;
        m_intermediateNavMeshCreateParams->walkableRadius = m_minimumObstacleClearance;
        m_intermediateNavMeshCreateParams->walkableClimb = m_maximumStepHeight;
        rcVcopy(m_intermediateNavMeshCreateParams->bmin, m_intermediatePolyMesh->bmin);
        rcVcopy(m_intermediateNavMeshCreateParams->bmax, m_intermediatePolyMesh->bmax);
        m_intermediateNavMeshCreateParams->cs = m_voxelSize;
        m_intermediateNavMeshCreateParams->ch = m_voxelHeight;
        m_intermediateNavMeshCreateParams->buildBvTree = true;
    }
    
    if (m_success)
    {
        m_outputNavMeshBuffer = 0;
        m_outputNavMeshBufferSize = 0;
        if (!dtCreateNavMeshData(m_intermediateNavMeshCreateParams, &m_outputNavMeshBuffer, &m_outputNavMeshBufferSize))
        {
            m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to create the detour navmesh data.");
            m_success = false;
        }
    }
    m_context->stopTimer(RC_TIMER_TOTAL);
    
    if (m_success)
    {
        duLogBuildTimes(*m_context, m_context->getAccumulatedTime(RC_TIMER_TOTAL));
    }
}

void NavMeshCreator::allocIntermediateResults()
{
    if (m_success)
    {
        if ((m_intermediateContourSet = rcAllocContourSet()) == 0)
        {
            m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to allocate memory for the contour set.");
            m_success = false;
        }
        else if ((m_intermediatePolyMesh = rcAllocPolyMesh()) == 0)
        {
            m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to allocate memory for the polygon mesh.");
            m_success = false;
        }
        else if ((m_intermediateHeightfield = rcAllocHeightfield()) == 0)
        {
            m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to allocate memory for the heightfield.");
            m_success = false;
        }
        else if ((m_intermediateCompactHeightfield = rcAllocCompactHeightfield()) == 0)
        {
            m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to allocate memory for the compact heightfield.");
            m_success = false;
        }
        else if ((m_intermediatePolyMeshDetail = rcAllocPolyMeshDetail()) == 0)
        {
            m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to allocate memory for the detail polygon mesh.");
            m_success = false;
        }
        else if ((m_intermediateTriangleTags = static_cast<unsigned char*>(rcAlloc(sizeof(unsigned char) * m_inputTrianglesCount,RC_ALLOC_TEMP))) == 0)
        {
            m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to allocate memory for the triangle tags.");
            m_success = false;
        }
        else if ((m_intermediateNavMeshCreateParams = static_cast<dtNavMeshCreateParams*>(rcAlloc(sizeof(dtNavMeshCreateParams),RC_ALLOC_TEMP)))==0)
        {
            m_context->log(RC_LOG_ERROR, "NavMeshCreator: unable to allocate memory for the Detour navmesh creation parameters.");
            m_success = false;
        }
    }
}

void NavMeshCreator::freeIntermediateResults()
{
    rcFreeContourSet(m_intermediateContourSet);
    m_intermediateContourSet = 0;
    
    rcFreePolyMesh(m_intermediatePolyMesh);
    m_intermediatePolyMesh = 0;
    
    rcFreeHeightField(m_intermediateHeightfield);
    m_intermediateHeightfield = 0;
    
    rcFreeCompactHeightfield(m_intermediateCompactHeightfield);
    m_intermediateCompactHeightfield = 0;
    
    rcFreePolyMeshDetail(m_intermediatePolyMeshDetail);
    m_intermediatePolyMeshDetail = 0;
}

OffMeshConnectionCreator::OffMeshConnectionCreator()
{
	memset(vert, 0, sizeof(float) * 6 * 100);
	memset(radius, 0, sizeof(float) * 100);
	memset(bidir, 0, sizeof(unsigned char) * 100);
	memset(areas, 0, sizeof(unsigned char) * 100);
	memset(flags, 0, sizeof(unsigned short) * 100);
	memset(ids, 0, sizeof(unsigned) * 100);

	count = 0;
}
