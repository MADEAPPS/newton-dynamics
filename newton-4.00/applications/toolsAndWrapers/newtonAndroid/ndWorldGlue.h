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

class ndWorldGlue : public ndWorld
{
	public:
	ndWorldGlue()
		:ndWorld()
	{
	}

	~ndWorldGlue()
	{
	}

	void SetSubSteps(int i)
	{
		ndWorld::SetSubSteps(i);
	}

	virtual void AddBody(ndRigidBodyGlue* const body)
	{
		ndWorld::AddBody(body->m_body);
	}

	virtual void RemoveBody(ndRigidBodyGlue* const body)
	{
		ndWorld::RemoveBody(body->m_body);
	}

	virtual void Update(float timestep)
	{
		ndWorld::Update(timestep);
	}
};

#endif 

