/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __dgAIWorld__
#define __dgAIWorld__

#include "dgAIAgent.h"

class dgAIAgentAgentTransition
{
};

class dgAIAgentAgentState
{
	public:
	dgAIAgentAgentState()
		:m_agent(NULL)
	{
	}

	~dgAIAgentAgentState()
	{
		dgAssert (m_agent);
		delete m_agent;
	}

	dgAIAgent* m_agent;
};


class dgAIAgentGraph: public dgGraph<dgAIAgentAgentState, dgAIAgentAgentTransition> 
{
	public: 
	dgAIAgentGraph (dgMemoryAllocator* const allocator);
	~dgAIAgentGraph();

	dgAIAgent* CreateAgent ();
	void DestroyAgent (dgAIAgent* const agent);

	void Update (dgFloat32 timestep);
};

DG_MSC_VECTOR_ALIGMENT
class dgAIWorld: public dgAIAgentGraph
{
	public:
	DG_CLASS_ALLOCATOR(allocator)
	dgAIWorld(dgMemoryAllocator* const allocator);
	virtual ~dgAIWorld(void);

	dgMemoryAllocator* GetAllocator() const;

	void* GetUserData() const;
	void SetUserData (void* const userData);

	dgAIAgent* GetGameStateAgent();


	void Update (dgFloat32 timestep);



	protected:
	dgAIAgent m_gameState;
	void* m_userData;
	dgMemoryAllocator* m_allocator;

} DG_GCC_VECTOR_ALIGMENT ;

#endif