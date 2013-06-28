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

#ifndef DETOURPATHQUEUE_H
#define DETOURPATHQUEUE_H

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

static const unsigned int DT_PATHQ_INVALID = 0;

typedef unsigned int dtPathQueueRef;

/// A path queue is a succession of destination in order to reach a specific location
class dtPathQueue
{
	/// The query to create a path of polygons between two points
	struct PathQuery
	{
		dtPathQueueRef ref;
		
		float startPos[3], endPos[3];	///< Path find start and end location.
		dtPolyRef startRef, endRef;		///< Path find start and end polygons.
		
		dtPolyRef* path;				///< Path results.
		int npath;						///< Number of polygon in the path.
		
		dtStatus status;				///< State of the query.
		int keepAlive;					///< Number of ticks during which the query has been kept alive.
		const dtQueryFilter* filter;	///< TODO: This is potentially dangerous!
	};
	
	static const int MAX_QUEUE = 8;		///< Maximal number of queue
	PathQuery m_queue[MAX_QUEUE];		///< The queues
	dtPathQueueRef m_nextHandle;		
	int m_maxPathSize;					///< Maximum size for a path
	int m_queueHead;					///< Used to navigate through the queues
	dtNavMeshQuery* m_navquery;			///< Used to perform queries on the navigation mesh
	
	/// Cleans the path queue
	void purge();
	
public:
	dtPathQueue();
	~dtPathQueue();
	
	/// Initializes the path queue (queries and navigation mesh)
	///
	/// @param[in]	maxPathSize				Maximum size for a path
	/// @param[in]	maxSearchNodeCount		Maximum number of search nodes
	/// @param[in]	nav						The navigation mesh
	///
	/// @return True if the initialization succeeded, false otherwise
	bool init(const int maxPathSize, const int maxSearchNodeCount, const dtNavMesh* nav);
	
	/// Updates the path request until there is nothing to update or until maxIters pathfinder iterations has been consumed.
	///
	/// @param[in]	maxIters	The maximal number of iterations allowed to update the path request
	void update(const int maxIters);
	
	/// Requests a path between the given points.
	///
	/// @param[in]	startRef	The polygon for the start point
	/// @param[in]	endRef		The polygon for the destination point
	/// @param[in]	startPos	The start position
	/// @param[in]	endPos		The destination position
	/// @param[in]	filter		The query filter
	///
	/// @return	Returns a reference on the path query of the newly created path
	dtPathQueueRef request(dtPolyRef startRef, dtPolyRef endRef,
						   const float* startPos, const float* endPos, 
						   const dtQueryFilter* filter);
	
	/// @name Data access
	/// @{
	/// Returns the status of the given query
	dtStatus getRequestStatus(dtPathQueueRef ref) const;
	
	/// Copies the given path into the one referenced by #ref.
	///
	/// @param[in]	ref			The reference of the path into which we want to copy the data
	/// @param[in]	path		The new path
	/// @param[out]	pathSize	The size of the new path
	/// @param[in]	maxPath		Maximum number of path results
	///
	/// @return	Returns DT_SUCCESS if the operation succeeded, DT_FAILURE otherwise
	dtStatus getPathResult(dtPathQueueRef ref, dtPolyRef* path, int* pathSize, const int maxPath);
	
	inline const dtNavMeshQuery* getNavQuery() const { return m_navquery; }
	/// @}

};

#endif // DETOURPATHQUEUE_H
