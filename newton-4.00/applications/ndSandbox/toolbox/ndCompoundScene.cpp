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
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsWorld.h"
#include "ndCompoundScene.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

static void AddBoxSubShape(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance, ndDemoEntity* const rootEntity, const dMatrix& location)
{
	ndShapeInstance box(new ndShapeBox(0.25f, 4.0f, 0.25f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);

	dMatrix matrix(location);
	matrix.m_posit.m_y += 1.9f;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);
	entity->SetMesh(geometry, dGetIdentityMatrix());
	geometry->Release();

	ndShapeMaterial material(box.GetMaterial());
	material.m_data.m_userData = entity;
	box.SetMaterial(material);

	box.SetLocalMatrix(matrix);
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->AddCollision(&box);

	//ndShapeCompound::ndTreeArray::dNode* const node = compound->AddCollision(&heighfieldInstance);
	//ndShapeInstance* const subInstance = node->GetInfo()->GetShape();
	//return subInstance;
}

static void AddSpeedBumpsSubShape(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance, ndDemoEntity* const rootEntity, const dMatrix& location, dInt32 count)
{
	ndShapeInstance capsule(new ndShapeCapsule(0.75f, 0.75f, 10.0f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &capsule, "Concrete_011_COLOR.tga", "Concrete_011_COLOR.tga", "Concrete_011_COLOR.tga", 1.0f, uvMatrix);

	dFloat32 spacing = 3.0f;
	dMatrix matrix(location);
	matrix.m_posit.m_y += -0.6f;
	matrix.m_posit.m_z -= (count/2) * spacing;
	ndShapeMaterial material(capsule.GetMaterial());
	for (dInt32 i = 0; i < count; i++)
	{
		ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);
		entity->SetMesh(geometry, dGetIdentityMatrix());

		material.m_data.m_userData = entity;
		capsule.SetMaterial(material);
		capsule.SetLocalMatrix(matrix);

		ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
		compound->AddCollision(&capsule);

		matrix.m_posit.m_z += spacing;
	}
	geometry->Release();
}

static void AddStaticMesh(ndDemoEntityManager* const scene, const char* const meshName, ndShapeInstance& sceneInstance, ndDemoEntity* const rootEntity, const dMatrix& location)
{
	fbxDemoEntity* const entity = scene->LoadFbxMesh(meshName);
	entity->Attach(rootEntity);
	entity->ResetMatrix(location);

	dPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	
	dInt32 stack = 1;
	dMatrix matrixBuffer[1024];
	fbxDemoEntity* entBuffer[1024];
	
	entBuffer[0] = entity;
	matrixBuffer[0] = entity->GetCurrentMatrix().Inverse();
	
	while (stack)
	{
		stack--;
		fbxDemoEntity* const ent = entBuffer[stack];
		dMatrix matrix(ent->GetCurrentMatrix() * matrixBuffer[stack]);
	
		if (ent->m_fbxMeshEffect)
		{
			dInt32 vertexStride = ent->m_fbxMeshEffect->GetVertexStrideInByte() / sizeof(dFloat64);
			const dFloat64* const vertexData = ent->m_fbxMeshEffect->GetVertexPool();
	
			dInt32 mark = ent->m_fbxMeshEffect->IncLRU();
			dPolyhedra::Iterator iter(*ent->m_fbxMeshEffect);
	
			dVector face[256];
			dMatrix worldMatrix(ent->GetMeshMatrix() * matrix);
			for (iter.Begin(); iter; iter++)
			{
				dEdge* const edge = &(*iter);
				if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
				{
					dInt32 count = 0;
					dEdge* ptr = edge;
					do
					{
						dInt32 i = ptr->m_incidentVertex * vertexStride;
						dVector point(dFloat32(vertexData[i + 0]), dFloat32(vertexData[i + 1]), dFloat32(vertexData[i + 2]), dFloat32(1.0f));
						face[count] = worldMatrix.TransformVector(point);
						count++;
						ptr->m_mark = mark;
						ptr = ptr->m_next;
					} while (ptr != edge);
	
					dInt32 materialIndex = ent->m_fbxMeshEffect->GetFaceMaterial(edge);
					meshBuilder.AddFace(&face[0].m_x, sizeof(dVector), 3, materialIndex);
				}
			}
		}
	
		for (fbxDemoEntity* child = (fbxDemoEntity*)ent->GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
		{
			entBuffer[stack] = child;
			matrixBuffer[stack] = matrix;
			stack++;
		}
	}
	meshBuilder.End(true);
	ndShapeInstance shape(new ndShapeStatic_bvh(meshBuilder));

	ndShapeMaterial material(shape.GetMaterial());
	material.m_data.m_userData = entity;
	shape.SetMaterial(material);
	
	shape.SetLocalMatrix(location);
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->AddCollision(&shape);
}

ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const dMatrix& location)
{
	ndDemoEntity* const rootEntity = new ndDemoEntity(location, nullptr);
	scene->AddEntity(rootEntity);

	ndShapeInstance sceneInstance (new ndShapeCompound());
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();

	dMatrix subShapeLocation(dGetIdentityMatrix());
	AddStaticMesh(scene, "playerarena.fbx", sceneInstance, rootEntity, subShapeLocation);
	//AddStaticMesh(scene, "flatplane.fbx", sceneInstance, rootEntity, subShapeLocation);

	subShapeLocation.m_posit.m_x += 10.0f;
	AddSpeedBumpsSubShape(scene, sceneInstance, rootEntity, subShapeLocation, 14);
	
	subShapeLocation.m_posit.m_z -= 15.0f;
	AddBoxSubShape(scene, sceneInstance, rootEntity, subShapeLocation);
	
	subShapeLocation.m_posit.m_z += 30.0f;
	AddBoxSubShape(scene, sceneInstance, rootEntity, subShapeLocation);
	
	subShapeLocation.m_posit.m_z += 5.0f;
	AddBoxSubShape(scene, sceneInstance, rootEntity, subShapeLocation);

	//subShapeLocation.m_posit.m_x = -200.0f;
	//subShapeLocation.m_posit.m_z = -200.0f;
	//AddHeightfieldSubShape(scene, sceneInstance, rootEntity, subShapeLocation);
	compound->EndAddRemove();

	ndPhysicsWorld* const world = scene->GetWorld();
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, rootEntity));
	body->SetMatrix(location);
	body->SetCollisionShape(sceneInstance);

	world->AddBody(body);
	return body;
}