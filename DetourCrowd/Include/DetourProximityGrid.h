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

#ifndef DETOURPROXIMITYGRID_H
#define DETOURPROXIMITYGRID_H


/// A proximity grid represents a grid where the user can add obstacles and agents.
/// These data can then be accessed and used for things like path following or collision avoidance
class dtProximityGrid
{
	float m_cellSize;			///< The size of a cell
	float m_invCellSize;		///< The inverse of the size of a cell
	
	/// Represents an item in the proximity grid
	struct Item
	{
		unsigned short id;		///< The id of the item
		short x, y;				///< The position of the item
		unsigned short next;
	};

	Item* m_pool;				///< A pool of item
	int m_poolHead;				///< The position in the pool (used to know whether we have reached the end or not)
	int m_poolSize;				///< The size of the pool
	
	unsigned short* m_buckets;	///< Hash buckets
	int m_bucketsSize;			///< Number of buckets for the hash table
	
	int m_bounds[4];			///< The bounds of the proximity grid (minX, minY, maxX, maxY)
	
public:
	dtProximityGrid();
	~dtProximityGrid();
	
	/// Initializes the proximity grid.
	///
	/// @param[in]	maxItems	The maximum number of item
	/// @param[in]	cellSize	The maximum size of a cell
	///
	/// @return True if the initialization was successful, false otherwise.
	bool init(const int maxItems, const float cellSize);
	
	/// Clears the proximity grid.
	/// Every items the grid contained is deleted.
	void clear();
	
	/// Adds an item to the proximity grid
	/// The id and the size of the item is provided.
	///
	/// @param[in]	id		The id of the new item
	/// @param[in]	minx	Minimum X point
	/// @param[in]	miny	Minimum Y point
	/// @param[in]	maxx	Maximum X point
	/// @param[in]	maxy	Maximum Y point
	void addItem(const unsigned short id,
				 const float minx, const float miny,
				 const float maxx, const float maxy);
	
	/// Gets the items contained within the given range
	///
	/// @param[in]	minx	Minimum X point
	/// @param[in]	miny	Minimum Y point
	/// @param[in]	maxx	Maximum X point
	/// @param[in]	maxy	Maximum Y point
	/// @param[out]	ids		The ids of the items found
	/// @param[in]	maxIds	The maximum number of item that can be found
	///
	/// @return Returns the number of item found
	int queryItems(const float minx, const float miny,
				   const float maxx, const float maxy,
				   unsigned short* ids, const int maxIds) const;
	
	/// @name Data access
	/// @{

	/// Return the number of items located at the given position
	///
	/// @param[in]	x	The X position
	/// @param[in]	y	The Y position
	///
	/// @return The number of item found
	int getItemCountAt(const int x, const int y) const;
	
	inline const int* getBounds() const { return m_bounds; }
	inline const float getCellSize() const { return m_cellSize; }
	/// @}
};

dtProximityGrid* dtAllocProximityGrid();
void dtFreeProximityGrid(dtProximityGrid* ptr);


#endif // DETOURPROXIMITYGRID_H

