/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _ND_WORLD_GLUE_H_
#define _ND_WORLD_GLUE_H_

#include "ndRigidBodyGlue.h"

class ndWorldGlue : public ndContainersFreeListAlloc<ndWorldGlue>
{
	public:
	ndWorldGlue()
		:ndContainersFreeListAlloc<ndWorldGlue>()
		,m_world(new ndWorld())
	{
	}

	~ndWorldGlue()
	{
	}

	void Sync()
	{
		m_world->Sync();
	}

	void SetSubSteps(int i)
	{
		m_world->SetSubSteps(i);
	}

	virtual void AddBody(ndRigidBodyGlue* const body)
	{
		m_world->AddBody(*(body->m_body));
	}

	virtual void RemoveBody(ndRigidBodyGlue* const body)
	{
		m_world->RemoveBody(*(body->m_body));
	}

	virtual void Update(float timestep)
	{
		m_world->Update(timestep);
	}

	ndSharedPtr<ndWorld> m_world;
};

#endif 

