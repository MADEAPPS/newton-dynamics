/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __ND_UIENTITY_H__
#define __ND_UIENTITY_H__

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

class ndDemoEntityManager;

class ndUIEntity: public ndDemoEntity
{
	public:
	ndUIEntity(ndDemoEntityManager* const scene);
	virtual ~ndUIEntity();

	virtual void RenderUI() = 0;
	virtual void RenderHelp() = 0;

	ndDemoEntityManager* m_scene;
};

#endif