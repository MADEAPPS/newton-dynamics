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
#include "DemoMesh.h"
#include "DemoInstanceEntity.h"


DemoInstanceEntity::DemoInstanceEntity(const DemoInstanceEntity& copyFrom)
	:DemoEntity(copyFrom)
{
	dAssert(0);
}

DemoInstanceEntity::DemoInstanceEntity(const dMatrix& matrix, DemoEntity* const parent)
	:DemoEntity(matrix, parent)
{
}

DemoInstanceEntity::~DemoInstanceEntity(void)
{
}

void DemoInstanceEntity::Render(dFloat timeStep, DemoEntityManager* const scene) const
{
//	DemoEntity::Render(timeStep, scene);

	glPushMatrix();

	// Set The matrix for this entity Node
	glMultMatrix(&m_matrix[0][0]);
	for (DemoEntity* child = GetChild(); child; child = child->GetSibling()) {
		glPushMatrix();
		dMatrix matrix(m_meshMatrix * child->GetRenderMatrix());
		glMultMatrix(&matrix[0][0]);
		m_mesh->Render(scene);
		glPopMatrix();
	}

	glPopMatrix();
}
