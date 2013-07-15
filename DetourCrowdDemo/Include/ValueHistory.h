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

#ifndef VALUEHISTORY_H
#define VALUEHISTORY_H

class ValueHistory
{
	static const int MAX_HISTORY = 256;
	float m_samples[MAX_HISTORY];
	int m_hsamples;
public:
	ValueHistory();
	~ValueHistory();

	inline void addSample(const float val)
	{
		m_hsamples = (m_hsamples+MAX_HISTORY-1) % MAX_HISTORY;
		m_samples[m_hsamples] = val;
	}
	
	inline int getSampleCount() const
	{
		return MAX_HISTORY;
	}
	
	inline float getSample(const int i) const
	{
		return m_samples[(m_hsamples+i) % MAX_HISTORY];
	}
	
	float getSampleMin() const;
	float getSampleMax() const;
	float getAverage() const;
};

struct GraphParams
{
	void setRect(int ix, int iy, int iw, int ih, int ipad);
	void setValueRange(float ivmin, float ivmax, int indiv, const char* iunits);
	
	int x, y, w, h, pad;
	float vmin, vmax;
	int ndiv;
	char units[16];
};

void drawGraphBackground(const GraphParams* p);

void drawGraph(const GraphParams* p, const ValueHistory* graph,
			   int idx, const char* label, const unsigned int col);


#endif // VALUEHISTORY_H