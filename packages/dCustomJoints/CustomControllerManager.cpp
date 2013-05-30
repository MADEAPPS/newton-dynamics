/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// CustomControllerManager.h: interface for the CustomControllerManager class.
//
//////////////////////////////////////////////////////////////////////


#include "CustomJointLibraryStdAfx.h"
#include "CustomControllerManager.h"



//dRttiRootClassSupportImplement(CustomControllerBase);



CustomControllerBase::CustomControllerBase(NewtonWorld* const world, const char* const managerName)
	:m_curTimestep(0.0f)
	,m_world(world)
{
	NewtonWorldAddPreListener (world, managerName, this, PreUpdate, NULL);
	NewtonWorldAddPostListener (world, managerName, this, PostUpdate, Destroy);
}


CustomControllerBase::~CustomControllerBase()
{
}

void* CustomControllerBase::operator new (size_t size)
{
	return NewtonAlloc(int (size));
}

void CustomControllerBase::operator delete (void* ptr)
{
	NewtonFree(ptr);
}



void CustomControllerBase::PreUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	CustomControllerBase* const me = (CustomControllerBase*) listenerUserData;
	dAssert (me->m_world == world);
	me->m_curTimestep = timestep;
	me->PreUpdate(timestep);
}

void CustomControllerBase::PostUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	CustomControllerBase* const me = (CustomControllerBase*) listenerUserData;
	dAssert (me->m_world == world);
	me->m_curTimestep = timestep;
	me->PostUpdate(timestep);
}



void CustomControllerBase::Destroy (const NewtonWorld* const world, void* const listenerUserData)
{
	CustomControllerBase* const me = (CustomControllerBase*) listenerUserData;
	delete me;
}


void* CustomControllerBase::AllocController (int size) const
{
	return NewtonAlloc(size);
}

void CustomControllerBase::FreeController (void* const ptr) const
{
	NewtonFree(ptr);
}
