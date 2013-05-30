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

#ifndef DETOURPARAMETRICBEHAVIOR_H
#define DETOURPARAMETRICBEHAVIOR_H

#include "DetourAlloc.h"
#include "DetourCrowd.h"
#include "DetourBehavior.h"

#include <new>


struct NoData
{};

template <typename T = NoData>
class dtParametrizedBehavior : public dtBehavior
{
public:
	dtParametrizedBehavior(unsigned nbMaxAgentsParams);

	virtual ~dtParametrizedBehavior();

	/// Adds a behavior parameters structure for the given agent and returns a pointer to it.
	/// If either the id of the agent is invalid or a parameter already exists for this agent, it return a NULL pointer.
	/// @param[in]	id 	The id of the agent we must add a parameter for
	T* addBehaviorParams(int id);

	/// Returns the behavior parameter for the given agent. NULL if it doesn't exist.
	T* getBehaviorParams(int id) const;

protected:
	struct Node
	{
		T value;
		Node* next;
		int id; // Mandatory to handle collisions
	};

	unsigned m_size;
	Node* m_agentsParams;
};


template <typename T>
dtParametrizedBehavior<T>::dtParametrizedBehavior(unsigned nbMaxAgentsParams)
	: m_size(nbMaxAgentsParams)
{
	if (m_size == 0)
		m_agentsParams = 0;
	else
	{
		m_agentsParams = (Node*) dtAlloc(sizeof(Node) * m_size, DT_ALLOC_PERM);

		for (unsigned i = 0; i < m_size; ++i)
		{
			new(&m_agentsParams[i]) Node();
			m_agentsParams[i].next = 0;
			m_agentsParams[i].id = -1;
		}
	}
}

template <typename T>
dtParametrizedBehavior<T>::~dtParametrizedBehavior()
{
	for (unsigned i = 0; i < m_size; ++i)
	{
		Node* n = &m_agentsParams[i];

		while (n->next)
		{
			Node* toDelete = n->next;
			n->next = n->next->next;
			dtFree(toDelete);
		}
	}

	dtFree(m_agentsParams);
	m_agentsParams = 0;
}

template <typename T>
T* dtParametrizedBehavior<T>::getBehaviorParams(int id) const
{
	if (id < 0 || m_size == 0)
		return 0;

	unsigned index = id % m_size;

	if (index >= m_size)
		return 0;

	Node* head = &m_agentsParams[index];

	if (head->id == id)
		return &head->value;
	
	while (head->next)
	{
		head = head->next;

		if (head->id == id)
			return &head->value;
	}

	return 0;
}

template <typename T>
T* dtParametrizedBehavior<T>::addBehaviorParams(int id)
{
	if (id < 0 || m_size == 0)
		return 0;

	unsigned index = id % m_size;

	if (index >= m_size)
		return 0;

	Node* head = &m_agentsParams[index];

	if (head->id == id)
		return 0;

	if (head->id == -1)
	{
		head->id = id;
		return &head->value;
	}

	while (head->next)
	{
		head = head->next;

		if (head->id == id)
			return 0;
	}

	void* mem = dtAlloc(sizeof(Node), DT_ALLOC_PERM);

	if (mem == 0)
		return 0;

	Node* newNode = new(mem) Node;
	
	newNode->next = 0;
	newNode->id = id;
	head->next = newNode;

	return &newNode->value;
}


#endif