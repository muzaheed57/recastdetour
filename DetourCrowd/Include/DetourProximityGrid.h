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

class dtProximityGrid
{
	int m_maxItems;				///< The max number of item in the pool
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
	
	bool init(const int maxItems, const float cellSize);
	
	void clear();
	
	void addItem(const unsigned short id,
				 const float minx, const float miny,
				 const float maxx, const float maxy);
	
	int queryItems(const float minx, const float miny,
				   const float maxx, const float maxy,
				   unsigned short* ids, const int maxIds) const;
	
	int getItemCountAt(const int x, const int y) const;
	
	inline const int* getBounds() const { return m_bounds; }
	inline const float getCellSize() const { return m_cellSize; }
};

dtProximityGrid* dtAllocProximityGrid();
void dtFreeProximityGrid(dtProximityGrid* ptr);


#endif // DETOURPROXIMITYGRID_H

