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
#include "ndLoadFbxMesh.h"
#include "ndDemoEntityManager.h"

ndBodyKinematic* BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName)
{
	fbxDemoEntity* const entity = LoadFbxMesh(scene, meshName);
	scene->AddEntity(entity);

	//ndPhysicsWorld* const world = scene->GetWorld();
	//dVector floor[] =
	//{
	//	{ 100.0f, 0.0f,  100.0f, 1.0f },
	//	{ 100.0f, 0.0f, -100.0f, 1.0f },
	//	{ -100.0f, 0.0f, -100.0f, 1.0f },
	//	{ -100.0f, 0.0f,  100.0f, 1.0f },
	//};
	//dInt32 index[][3] = { { 0, 1, 2 },{ 0, 2, 3 } };
	//
	//dPolygonSoupBuilder meshBuilder;
	//meshBuilder.Begin();
	//meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[0][0], 3);
	//meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[1][0], 3);
	//meshBuilder.End(true);
	//
	//ndShapeInstance box(new ndShapeStaticBVH(meshBuilder));
	//dMatrix uvMatrix(dGetIdentityMatrix());
	//uvMatrix[0][0] *= 0.025f;
	//uvMatrix[1][1] *= 0.025f;
	//uvMatrix[2][2] *= 0.025f;
	//ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);
	//
	//dMatrix matrix(dGetIdentityMatrix());
	//matrix.m_posit.m_y = -0.5f;
	//ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	//entity->SetMesh(geometry, dGetIdentityMatrix());
	//
	//ndBodyDynamic* const body = new ndBodyDynamic();
	//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	//body->SetMatrix(matrix);
	//body->SetCollisionShape(box);
	//
	//world->AddBody(body);
	//
	//scene->AddEntity(entity);
	//geometry->Release();

	entity->CleanIntermediate();
	return nullptr;
}