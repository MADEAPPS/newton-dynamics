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
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"

void BuildStaticMap(ndDemoEntityManager* const scene, const char* const meshName)
{
	char outPathName[1024];
	dGetWorkingFileName(meshName, outPathName);

	FILE* fp = fopen(outPathName, "rb");
	if (!fp)
	{
		dAssert(0);
		return;
	}

	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	ofbx::u8* content = new ofbx::u8[file_size];
	fread(content, 1, file_size, fp);
	ofbx::IScene* const fbxScene = ofbx::load((ofbx::u8*)content, file_size, (ofbx::u64)ofbx::LoadFlags::TRIANGULATE);
	fbxScene->destroy();
	delete[] content;


	ndPhysicsWorld* const world = scene->GetWorld();
	dVector floor[] =
	{
		{ 100.0f, 0.0f,  100.0f, 1.0f },
		{ 100.0f, 0.0f, -100.0f, 1.0f },
		{ -100.0f, 0.0f, -100.0f, 1.0f },
		{ -100.0f, 0.0f,  100.0f, 1.0f },
	};
	dInt32 index[][3] = { { 0, 1, 2 },{ 0, 2, 3 } };
	//dInt32 index[][4] = {{ 0, 1, 2, 3 }};

	dPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[0][0], 3);
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[1][0], 3);
	//meshBuilder.AddFace(&floor[0].m_x, sizeof(dVector), 4, 31);
	meshBuilder.End(true);

	ndShapeInstance box(new ndShapeStaticBVH(meshBuilder));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = -0.5f;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);

	world->AddBody(body);

	scene->AddEntity(entity);
	geometry->Release();

	//fbxScene->destroy();
}

