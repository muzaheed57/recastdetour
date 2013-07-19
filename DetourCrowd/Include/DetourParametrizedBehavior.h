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

#ifndef DETOURPARAMETRIZEDBEHAVIOR_H
#define DETOURPARAMETRIZEDBEHAVIOR_H

#include "DetourAlloc.h"
#include "DetourCrowd.h"
#include "DetourBehavior.h"

#include <climits>
#include <new>


/// Empty data structure used when no parameters are required
/// @ingroup behavior
struct NoData
{};

/// A behavior that can be parametrized.
///
/// Each agents can have parameters related to the behaviors
/// @ingroup behavior
template <typename T = NoData>
class dtParametrizedBehavior : public dtBehavior
{
public:
	/// Constructs the behavior
	/// The user must specify how many agents will be using this behavior.
	/// You can have have a number of agents greater than this number, 
	/// but it will be slower (because of the collisions in the Hash Table)
	///
	/// @param[in]	nbAgentsEstimated	An estimation of the number of agents that will be using this behavior. 
	///									You have more agents than this number indicates in the end, or less. It is just an estimation
	explicit dtParametrizedBehavior(unsigned nbAgentsEstimated);
	virtual ~dtParametrizedBehavior();

	/// Adds a behavior parameters structure for the given agent and returns a pointer to it.
	/// If the parameters already existed, then it is returned.
	/// If either the id of the agent is invalid or a parameter already exists for this agent, it return a NULL pointer.
	/// @param[in]	id 	The id of the agent we must add a parameter for
	/// Returns the behavior parameter for the given agent. NULL if it doesn't exist.
	T* getBehaviorParams(unsigned id) const;

	/// This method automatically gets the parameters of the given agents and perform some checks on them (do they exist?). 
	/// It then calls the `dtParametrizedBehavior::doUpdate()` method, which contains the implementation of the behavior.
	/// This is the method the user must call from its main loop, but not the one he should overload.
	virtual void update(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt);

protected:
	/// This method will eventually be called by the `dtParametrizedBehavior::update()` method. 
	/// This is the method the user must overload in order to describe his behavior, 
	/// but not the one he should call (since this method is called by the `dtParametrizedBehavior::update()` method)
	/// @param[in]	query			Allows the user to query data from the crowd.
	/// @param[in]	oldAgent		The agent we want to update.
	/// @param[out]	newAgent		The agent storing the updated version of the oldAgent.
	/// @param[in]	currentParams	The parameters to the oldAgent.
	/// @param[out]	newParams		The parameters to the newAgent.
	/// @param[in]	dt				The time, in seconds, to update the simulation. [Limit: > 0, otherwise strange things can happen (undefined behavior)]
	virtual void doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
						  const T& currentParams, T& newParams, float dt) = 0;

	/// Linked List 
	struct Node
	{
		T value;		///< The parameter
		Node* next;		///< Pointer to the next element
		unsigned id;	///< The id of the agent owning this parameter. Mandatory to handle collisions
	};

	unsigned m_size;		///< Number of parameters at the beginning
	Node* m_agentsParams;	///< The data structure containing the parameters
};


template <typename T>
dtParametrizedBehavior<T>::dtParametrizedBehavior(unsigned preAllocationSize)
	: m_size(preAllocationSize)
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
			m_agentsParams[i].id = UINT_MAX;
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
			toDelete->~Node();
			dtFree(toDelete);
		}
	}

	dtFree(m_agentsParams);
	m_agentsParams = 0;
}

template <typename T>
T* dtParametrizedBehavior<T>::getBehaviorParams(unsigned id) const
{
	if (m_size == 0)
		return 0;

	unsigned index = id % m_size;

	if (index >= m_size)
		return 0;

	Node* head = &m_agentsParams[index];

	if (head->id == id && head->id < UINT_MAX)
		return &head->value;
	
	while (head->next)
	{
		head = head->next;

		if (head->id == id && head->id < UINT_MAX)
			return &head->value;
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

template <typename T>
void dtParametrizedBehavior<T>::update(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt)
{
	T* params = getBehaviorParams(oldAgent.id);

	if (!params)
		return;

	T newParams = *params;

	doUpdate(query, oldAgent, newAgent, *params, newParams, dt);

	*params = newParams;
}


#endif
