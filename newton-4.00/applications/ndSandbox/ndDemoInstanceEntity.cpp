/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndDemoMesh.h"
#include "ndDemoInstanceEntity.h"


ndDemoInstanceEntity::ndDemoInstanceEntity(const ndDemoInstanceEntity& copyFrom)
	:ndDemoEntity(copyFrom)
{
	dAssert(0);
}

ndDemoInstanceEntity::ndDemoInstanceEntity(const dMatrix& matrix, ndDemoEntity* const parent)
	:ndDemoEntity(matrix, parent)
{
}

ndDemoInstanceEntity::~ndDemoInstanceEntity(void)
{
}

void ndDemoInstanceEntity::Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrixIn) const
{
	dAssert(0);
	//glPushMatrix();
	//
	//// Set The matrix for this entity Node
	//glMultMatrix(&m_matrix[0][0]);
	//for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling()) {
	//	glPushMatrix();
	//	dMatrix matrix(m_meshMatrix * child->GetRenderMatrix());
	//	glMultMatrix(&matrix[0][0]);
	//	m_mesh->Render(scene, TODO);
	//	glPopMatrix();
	//}
	//
	//glPopMatrix();
}
