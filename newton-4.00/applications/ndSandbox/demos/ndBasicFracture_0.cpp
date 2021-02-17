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
#include "ndSkyBox.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndConvexFractureModel_0.h"


static void makePointCloud(const dVector& size, dInt32 count, dArray<dVector>& pointCloud)
{
	const dVector scale(0.2f);
	const dVector invScale(size * scale.Reciproc());
	const dVector pMin(size.Scale(-0.5f));
	for (dInt32 i = 0; i < count; i++)
	{
		dFloat32 x = dRand();
		dFloat32 y = dRand();
		dFloat32 z = dRand();
		dVector randPoint(x, y, z, dFloat32(0.0f));
		randPoint *= invScale;
		randPoint = pMin + scale * randPoint.Floor();
		pointCloud.PushBack(randPoint);
	}
}

class ndConvexFractureBox: public ndConvexFracture
{
	public:
	ndConvexFractureBox(ndDemoEntityManager* const scene)
		:ndConvexFracture()
	{
		dVector shapeBox(1.0f, 5.0f, 20.0f, 0.0f);
		ndShapeInstance shape(new ndShapeBox(shapeBox.m_x, shapeBox.m_y, shapeBox.m_z));
		m_singleManifoldMesh = new ndMeshEffect(shape);

		ndMeshEffect::dMaterial material;
		strcpy(material.m_textureName, "reljef.tga");
		m_singleManifoldMesh->GetMaterials().PushBack(ndMeshEffect::dMaterial());

		dMatrix textureMatrix(dGetIdentityMatrix());
		textureMatrix[0][0] = 1.0f;
		textureMatrix[1][1] = 1.0f;
		textureMatrix.m_posit.m_x = -0.5f;
		textureMatrix.m_posit.m_y = -0.5f;
		m_singleManifoldMesh->UniformBoxMapping(0, textureMatrix);

		makePointCloud(shapeBox, 20, m_pointCloud);

		m_textureMatrix = dGetIdentityMatrix();
		m_textureMatrix[0][0] = 1.0f;
		m_textureMatrix[1][1] = 1.0f;
		m_textureMatrix.m_posit.m_x = -0.5f;
		m_textureMatrix.m_posit.m_y = -0.5f;
		strcpy(material.m_textureName, "concreteBrick.tga");
		m_singleManifoldMesh->GetMaterials().PushBack(ndMeshEffect::dMaterial());

		m_interiorMaterialIndex = m_singleManifoldMesh->GetMaterials().GetCount() - 1;
		GenerateEffect(scene);
	}

	~ndConvexFractureBox()
	{
		delete m_singleManifoldMesh;
	}
};

void ndBasicFracture_0(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	//ndPhysicsWorld* const world = scene->GetWorld();
	//ndConvexFractureModel_1* const fractureManager = new ndConvexFractureModel_1(scene);
	//world->AddModel(fractureManager);
	//world->RegisterModelUpdate(fractureManager);
	//
	//dMatrix matrix(dGetIdentityMatrix());
	//
	//matrix.m_posit.m_x += 10.0f;
	//matrix.m_posit.m_y += 2.0f;
	//matrix.m_posit.m_z -= 5.0f;
	//AddBoxEffect(fractureManager, matrix);
	//
	//matrix.m_posit.m_z += 5.0f;
	//AddCapsuleEffect(fractureManager, matrix);
	//
	//matrix.m_posit.m_z += 5.0f;
	//AddCylinderEffect(fractureManager, matrix);

	//ndMeshEffect* const mesh = MakeBoxMesh();
	ndConvexFractureBox box(scene);


	

	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	//dVector origin(-60.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
