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

#ifndef NAVMESHCREATOR_H
#define NAVMESHCREATOR_H

class rcContext;
struct rcContourSet;
struct rcHeightfield;
struct rcCompactHeightfield;
struct rcPolyMesh;
struct rcPolyMeshDetail;
struct dtNavMeshCreateParams;

namespace area
{
    enum Type
    {
        Obstacle = 0,
        Ground
    };
}

namespace navigationFlags
{
    enum Type
    {
        NonWalkable = 0, //!< Unable to walk
        Walkable = 0x01, //!< Ability to walk (ground, grass, road, ..)
    };
}

// Structure used to create offMesh connections before the creation of the navmesh
struct OffMeshConnectionCreator
{
	OffMeshConnectionCreator();

	float vert[6 * 100];		// start & end position [s1, s2, s3, e1, e2, e3]
	float radius[100];		
	unsigned char bidir[100];	// Is the connection bi directional?
	unsigned char areas[100];
	unsigned short flags[100];
	unsigned ids[100];			
	int count;
};

class NavMeshCreator
{
public:
    
    /** Init the parameters to default values.
    */
    void initParameters();
    
    /** Compute the navmesh
     */
    void computeNavMesh();
    
    void allocIntermediateResults();
    void freeIntermediateResults();
    
    
    rcContext* m_context;
    bool m_success;
    
    int m_inputVerticesCount; //!< Number of input vertices
    const float* m_inputVertices; //!< Input vertices (size = 3 * m_inputVerticesCount)
    int m_inputTrianglesCount; //!< Number of input triangles
    const int* m_inputTriangles; //!< Input vertices (size = 3 * m_inputTrianglesCount)
    float m_min[3]; //!< input geometry aabb min corner
    float m_max[3]; //!< input geometry aabb max corner
    
    float m_voxelSize;
    float m_voxelHeight;
    float m_minimumCeilingClearance;
    float m_maximumStepHeight;
    float m_minimumObstacleClearance;
    float m_maximumSlope; //!< The maximum navigable slope (in degrees)
    float m_edgeMaxError; //!< The maximum distance the contour should deviate from the raw border.
    float m_edgeMaxLength;//!< The maximum length of edges.
    int m_polyMaxNbVertices;
    float m_sampleDist; //!< The distance between ground samples of the detail mesh.
    float m_sampleMaxError; //!< The maximum distance the detail mesh surface should deviate from heightfield data.
    int m_regionMinSize;
    int m_regionMergeSize;
    
    unsigned char* m_intermediateTriangleTags;
    int m_intermediateHeightfieldWidth;
    int m_intermediateHeightfieldHeight;
    rcContourSet* m_intermediateContourSet;
    rcHeightfield* m_intermediateHeightfield;
    rcCompactHeightfield* m_intermediateCompactHeightfield;
    rcPolyMesh* m_intermediatePolyMesh;
    rcPolyMeshDetail* m_intermediatePolyMeshDetail;
    dtNavMeshCreateParams* m_intermediateNavMeshCreateParams;
    
    unsigned char* m_outputNavMeshBuffer;
    int m_outputNavMeshBufferSize;

	OffMeshConnectionCreator m_offMeshConnectionCreator;
};

#endif