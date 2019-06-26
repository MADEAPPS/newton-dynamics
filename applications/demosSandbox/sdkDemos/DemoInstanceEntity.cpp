/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "DemoInstanceEntity.h"


DemoInstanceEntity::DemoInstanceEntity(const DemoInstanceEntity& copyFrom)
	:DemoEntity(copyFrom)
{
	dAssert(0);
}

DemoInstanceEntity::DemoInstanceEntity(const dMatrix& matrix, DemoEntity* const parent)
	:DemoEntity(matrix, parent)
{
	dAssert(0);
}

DemoInstanceEntity::~DemoInstanceEntity(void)
{
	dAssert(0);
}

void DemoInstanceEntity::Render(dFloat timeStep, DemoEntityManager* const scene) const
{
	dAssert(0);
}
