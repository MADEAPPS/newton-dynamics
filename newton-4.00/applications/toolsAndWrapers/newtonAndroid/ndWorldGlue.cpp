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

#include "ndNewton.h"
#include "ndWorldGlue.h"
#include "ndRigidBodyGlue.h"
#include "ndBodyNotifyGlue.h"
#include "ndShapeInstanceGlue.h"

ndWorldGlue::ndWorldGlue()
	:ndContainersFreeListAlloc<ndWorldGlue>()
	,m_world(new ndWorld())
{
}

ndWorldGlue::~ndWorldGlue()
{
	delete m_world;
}

void ndWorldGlue::Sync()
{
	m_world->Sync();
}

void ndWorldGlue::SetSubSteps(int i)
{
	m_world->SetSubSteps(i);
}

void ndWorldGlue::AddBody(ndRigidBodyGlue* const body)
{
	m_world->AddBody(body->m_body);
	//ndBodyNotifyGlue* const notification = (ndBodyNotifyGlue*)body->m_body->GetNotifyCallback();
	ndBodyNotifyGlue* const notification = body->GetNotifyCallback();
	notification->m_world = this;
}

void ndWorldGlue::RemoveBody(ndRigidBodyGlue* const body)
{
	//ndBodyNotifyGlue* const notification = (ndBodyNotifyGlue*)body->m_body->GetNotifyCallback();
	ndBodyNotifyGlue* const notification = body->GetNotifyCallback();
	notification->m_world = nullptr;
	m_world->RemoveBody(body->m_body);
}

void ndWorldGlue::Update(float timestep)
{
	m_world->Update(timestep);
}

