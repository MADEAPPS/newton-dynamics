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

#include "ndSandboxStdafx.h"
#include "ndUIEntity.h"


ndUIEntity::ndUIEntity(ndDemoEntityManager* const scene)
	:ndDemoEntity(ndGetIdentityMatrix())
	,m_scene(scene)
{
}

ndUIEntity::~ndUIEntity()
{
}
