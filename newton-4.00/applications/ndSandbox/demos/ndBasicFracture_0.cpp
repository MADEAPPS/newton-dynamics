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
		m_mass = 100.0f;
		dVector shapeBox(1.0f, 5.0f, 20.0f, 0.0f);
		ndShapeInstance shape(new ndShapeBox(shapeBox.m_x, shapeBox.m_y, shapeBox.m_z));
		m_singleManifoldMesh = new ndMeshEffect(shape);

		ndMeshEffect::dMaterial& material0 = m_singleManifoldMesh->GetMaterials()[0];
		strcpy(material0.m_textureName, "reljef.tga");

		dMatrix textureMatrix(dGetIdentityMatrix());
		textureMatrix[0][0] = 1.0f / 8.0f;
		textureMatrix[1][1] = 1.0f / 4.0f;
		textureMatrix.m_posit.m_x = -0.5f;
		textureMatrix.m_posit.m_y = -0.5f;
		m_singleManifoldMesh->UniformBoxMapping(0, textureMatrix);

		//makePointCloud(shapeBox, 200, m_pointCloud);
		//makePointCloud(shapeBox, 20, m_pointCloud);

		m_textureMatrix = dGetIdentityMatrix();
		m_textureMatrix[0][0] = 1.0f;
		m_textureMatrix[1][1] = 1.0f;
		m_textureMatrix.m_posit.m_x = -0.5f;
		m_textureMatrix.m_posit.m_y = -0.5f;

		m_singleManifoldMesh->GetMaterials().PushBack(ndMeshEffect::dMaterial());
		ndMeshEffect::dMaterial& material1 = m_singleManifoldMesh->GetMaterials()[1];
		strcpy(material1.m_textureName, "concreteBrick.tga");

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

	ndConvexFractureBox fractureBox(scene);

	dMatrix matrix(dGetIdentityMatrix());
	matrix = dPitchMatrix(5.0f * dDegreeToRad) * matrix;
	matrix.m_posit.m_x += 10.0f;
	matrix.m_posit.m_y += 5.0f;
	fractureBox.AddEffect(scene, matrix);

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
