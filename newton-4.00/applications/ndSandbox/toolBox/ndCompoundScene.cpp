/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
//#include "ndDemoMesh.h"
//#include "ndDemoCamera.h"
//#include "ndPhysicsUtils.h"
//#include "ndDebugDisplay.h"
//#include "ndPhysicsWorld.h"
//#include "ndTargaToOpenGl.h"
#include "ndCompoundScene.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"


//ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const dMatrix& location)
ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const dMatrix& )
{
	//dArray<dVector> heightfield(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
	//MakeNoiseHeightfield(heightfield);
	//
	//// create the visual mesh
	//ndDemoMesh* const mesh = new ndHeightfieldMesh(heightfield, scene->GetShaderCache());
	//ndDemoEntity* const entity = new ndDemoEntity(location, nullptr);
	//entity->SetMesh(mesh, dGetIdentityMatrix());
	//
	//// create the height field collision and rigid body
	//ndShapeInstance heighfieldInstance(new ndShapeHeightfield(D_TERRAIN_WIDTH, D_TERRAIN_WIDTH,
	//	ndShapeHeightfield::m_invertedDiagonals,
	//	1.0f / 100.0f, D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE));
	//
	//ndShapeHeightfield* const shape = heighfieldInstance.GetShape()->GetAsShapeHeightfield();
	//dArray<dInt16>& hightMap = shape->GetElevationMap();
	//dAssert(hightMap.GetCount() == heightfield.GetCount());
	////ndShapeInfo hightInfo(heighfieldInstance.GetShapeInfo());
	////dInt16* const highMap = hightInfo.m_heightfield.m_elevation;
	//for (int i = 0; i < heightfield.GetCount(); i++)
	//{
	//	dFloat32 high = heightfield[i].m_y * 100.0f;
	//	dAssert(high < dFloat32(1 << 15));
	//	dAssert(high > dFloat32(-1 << 15));
	//	hightMap[i] = dInt16(high);
	//}
	//
	//shape->UpdateElevationMapAabb();
	//
	//dVector boxP0;
	//dVector boxP1;
	//// get the position of the aabb of this geometry
	//dMatrix entMatrix (entity->GetCurrentMatrix());
	//
	////NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x);
	//heighfieldInstance.CalculateAABB(entMatrix, boxP0, boxP1);
	//
	//entMatrix.m_posit = (boxP0 + boxP1).Scale (-0.5f);
	//entMatrix.m_posit.m_w = 1.0f;
	//
	//ndPhysicsWorld* const world = scene->GetWorld();
	//ndBodyDynamic* const body = new ndBodyDynamic();
	//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	//body->SetMatrix(location);
	//body->SetCollisionShape(heighfieldInstance);
	//
	//world->AddBody(body);
	//scene->AddEntity(entity);
	//mesh->Release();
	//return body;

	dMatrix heighfieldLocation(dGetIdentityMatrix());
	heighfieldLocation.m_posit.m_x = -200.0f;
	heighfieldLocation.m_posit.m_z = -200.0f;

	ndShapeInstance sceneInstance (new ndShapeCompound());

	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();

	ndBodyKinematic* const body = BuildHeightFieldTerrain(scene, heighfieldLocation);

	compound->EndAddRemove();
	return body;
}