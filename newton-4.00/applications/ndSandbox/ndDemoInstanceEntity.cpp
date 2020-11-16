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
	,m_instanceMesh(copyFrom.m_instanceMesh)
{
	dAssert(0);
	m_instanceMesh->AddRef();
}

ndDemoInstanceEntity::ndDemoInstanceEntity(ndDemoMeshIntance* const instanceMesh)
	:ndDemoEntity(dGetIdentityMatrix(), nullptr)
	,m_instanceMesh(instanceMesh)
{
	m_instanceMesh->AddRef();
}

ndDemoInstanceEntity::~ndDemoInstanceEntity(void)
{
	m_instanceMesh->Release();
}

void ndDemoInstanceEntity::Render(dFloat32 timestep, ndDemoEntityManager* const scene, const dMatrix& matrixIn) const
{
	//dMatrix nodeMatrix(m_matrix * matrixIn);
	//for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling())
	//{
	//	child->Render(timestep, scene, nodeMatrix);
	//}

	dMatrix nodeMatrix(m_matrix * matrixIn);
	dInt32 count = 0;
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling())
	{
		count++;
	}

	dInt32 index = 0;
	dVector* const matrixStack = dAlloca(dVector, count);
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling())
	{
		dMatrix matrix(child->GetCurrentMatrix());
		matrixStack[index] = matrix.m_posit;
		index++;
	}

	m_instanceMesh->SetParticles(count, &matrixStack[0]);
	m_instanceMesh->Render(scene, nodeMatrix);
}
