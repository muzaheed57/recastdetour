//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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

#ifndef DETOUTPATHCORRIDOR_H
#define DETOUTPATHCORRIDOR_H

#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"

/// Represents a dynamic polygon corridor used to plan agent movement.
/// @ingroup behavior, detour
class dtPathCorridor
{
	float m_pos[3];		///< The position of the agent in the corridor
	float m_target[3];	///< The destination of the agent in the corridor
	
	dtPolyRef* m_path;	///< The path as an array of polygon
	int m_npath;		///< The number of polygon in the path
	int m_maxPath;		///< The maximum path size the corridor can handle
	
public:
	dtPathCorridor();
	~dtPathCorridor();

	/// Copy constructor
	dtPathCorridor(const dtPathCorridor& o);

	/// Assignment operator
	dtPathCorridor& operator=(const dtPathCorridor& o);
	
	/// Allocates the corridor's path buffer. 
	///  @param[in]		maxPath		The maximum path size the corridor can handle.
	/// @return True if the initialization succeeded.
	bool init(const int maxPath);
	
	/// Resets the path corridor to the specified position.
	///  @param[in]		ref		The polygon reference containing the position.
	///  @param[in]		pos		The new position in the corridor. [(x, y, z)]
	void reset(dtPolyRef ref, const float* pos);
	
	/// Finds the corners in the corridor from the position toward the target. (The straightened path.)
	///  @param[out]	cornerVerts		The corner vertices. [(x, y, z) * cornerCount] [Size: <= maxCorners]
	///  @param[out]	cornerFlags		The flag for each corner. [(flag) * cornerCount] [Size: <= maxCorners]
	///  @param[out]	cornerPolys		The polygon reference for each corner. [(polyRef) * cornerCount] 
	///  								[Size: <= @p maxCorners]
	///  @param[in]		maxCorners		The maximum number of corners the buffers can hold.
	///  @param[in]		navquery		The query object used to build the corridor.
	///  @param[in]		filter			The filter to apply to the operation.
	/// @return The number of corners returned in the corner buffers. [0 <= value <= @p maxCorners]
	int findCorners(float* cornerVerts, unsigned char* cornerFlags,
					dtPolyRef* cornerPolys, const int maxCorners,
					const dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Attempts to optimize the path if the specified point is visible from the current position.
	///  @param[in]		next					The point to search toward. [(x, y, z])
	///  @param[in]		pathOptimizationRange	The maximum range to search. [Limit: > 0]
	///  @param[in]		navquery				The query object used to build the corridor.
	///  @param[in]		filter					The filter to apply to the operation.			
	void optimizePathVisibility(const float* next, const float pathOptimizationRange,
								const dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Attempts to optimize the path using a local area search. (Partial replanning.) 
	///  @param[in]		navquery	The query object used to build the corridor.
	///  @param[in]		filter		The filter to apply to the operation.	
	bool optimizePathTopology(dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Updates the corridor to move through an off-mesh connection
	///
	/// @param[in]	offMeshConRef	A reference to the off-mesh connection polygon
	/// @param[out]	refs			Array containing the reference polygon before the off mesh connection, 
	///								and a reference to the off mesh connection polygon
	/// @param[out]	startPos		The start position of the off-mesh connection. [(x, y, z)]
	/// @param[out]	endPos			The end position of the off-mesh connection. [(x, y, z)]
	/// @param[in]	navquery		The navigation mesh query
	/// @return	False if the #offMeshConRef could not be found or if the move could not be done. True otherwise.
	bool moveOverOffmeshConnection(dtPolyRef offMeshConRef, dtPolyRef* refs,
								   float* startPos, float* endPos,
								   const dtNavMeshQuery* navquery);

	/// Used to make sure the first polygon of the start is valid.
	/// Updates the position and the polygon of the first element of the path
	/// @param[in]	safeRef		The safe reference to the first polygon
	/// @param[in]	safePos		The safe start position. [(x, y, z)]
	/// @return Returns True
	bool fixPathStart(dtPolyRef safeRef, const float* safePos);

	/// Reconstructs the path so that all polygons can be valid
	///
	/// @param[in]	safeRef		The safe reference for the first polygon
	/// @param[in]	safePos		The safe start position. [(x, y, z)]
	/// @param[in]	navquery	The navigation mesh query
	/// @param[in]	filter		The Query Filter
	/// @returns True
	bool trimInvalidPath(dtPolyRef safeRef, const float* safePos,
						 dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Checks the current corridor path to see if its polygon references remain valid. 
	///  @param[in]		maxLookAhead	The number of polygons from the beginning of the corridor to search.
	///  @param[in]		navquery		The query object used to build the corridor.
	///  @param[in]		filter			The filter to apply to the operation.	
	bool isValid(const int maxLookAhead, const dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Moves the position from the current location to the desired location, adjusting the corridor 
	/// as needed to reflect the change.
	///  @param[in]		npos		The desired new position. [(x, y, z)]
	///  @param[in]		navquery	The query object used to build the corridor.
	///  @param[in]		filter		The filter to apply to the operation.
	void movePosition(const float* npos, const dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	/// Moves the target from the curent location to the desired location, adjusting the corridor
	/// as needed to reflect the change. 
	///  @param[in]		npos		The desired new target position. [(x, y, z)]
	///  @param[in]		navquery	The query object used to build the corridor.
	///  @param[in]		filter		The filter to apply to the operation.
	void moveTargetPosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Loads a new path and target into the corridor.
	///  @param[in]		target		The target location within the last polygon of the path. [(x, y, z)]
	///  @param[in]		path		The path corridor. [(polyRef) * @p npolys]
	///  @param[in]		npath		The number of polygons in the path.
	void setCorridor(const float* target, const dtPolyRef* polys, const int npath);
	
	/// Gets the current position within the corridor. (In the first polygon.)
	/// @return The current position within the corridor.
	inline const float* getPos() const { return m_pos; }

	/// Gets the current target within the corridor. (In the last polygon.)
	/// @return The current target within the corridor.
	inline const float* getTarget() const { return m_target; }
	
	/// The polygon reference id of the first polygon in the corridor, the polygon containing the position.
	/// @return The polygon reference id of the first polygon in the corridor. (Or zero if there is no path.)
	inline dtPolyRef getFirstPoly() const { return m_npath ? m_path[0] : 0; }

	/// The polygon reference id of the last polygon in the corridor, the polygon containing the target.
	/// @return The polygon reference id of the last polygon in the corridor. (Or zero if there is no path.)
	inline dtPolyRef getLastPoly() const { return m_npath ? m_path[m_npath-1] : 0; }
	
	/// The corridor's path.
	/// @return The corridor's path. [(polyRef) * #getPathCount()]
	inline const dtPolyRef* getPath() const { return m_path; }

	/// The number of polygons in the current corridor path.
	/// @return The number of polygons in the current corridor path.
	inline int getPathCount() const { return m_npath; }

	static const int MAX_VISITED = 16;
};

int dtMergeCorridorStartMoved(dtPolyRef* path, const int npath, const int maxPath,
							  const dtPolyRef* visited, const int nvisited);

int dtMergeCorridorEndMoved(dtPolyRef* path, const int npath, const int maxPath,
							const dtPolyRef* visited, const int nvisited);

int dtMergeCorridorStartShortcut(dtPolyRef* path, const int npath, const int maxPath,
								 const dtPolyRef* visited, const int nvisited);

#endif // DETOUTPATHCORRIDOR_H
