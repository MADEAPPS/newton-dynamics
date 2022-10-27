/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

static void AddBoxSubShape(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance, ndDemoEntity* const rootEntity, const ndMatrix& location)
{
	ndShapeInstance box(new ndShapeBox(0.25f, 4.0f, 0.25f));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix));

	ndMatrix matrix(location);
	matrix.m_posit.m_y += 1.9f;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);
	entity->SetMesh(geometry);

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

static void AddSpeedBumpsSubShape(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance, ndDemoEntity* const rootEntity, const ndMatrix& location, ndInt32 count)
{
	ndShapeInstance capsule(new ndShapeCapsule(0.75f, 0.75f, 10.0f));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndSharedPtr<ndDemoMeshInterface>geometry (new ndDemoMesh("box", scene->GetShaderCache(), &capsule, "Concrete_011_COLOR.tga", "Concrete_011_COLOR.tga", "Concrete_011_COLOR.tga", 1.0f, uvMatrix));

	ndFloat32 spacing = 3.0f;
	ndMatrix matrix(location);
	matrix.m_posit.m_y += -0.6f;
	matrix.m_posit.m_z -= (ndFloat32)(count/2) * spacing;
	ndShapeMaterial material(capsule.GetMaterial());
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);
		entity->SetMesh(geometry);

		material.m_data.m_userData = entity;
		capsule.SetMaterial(material);
		capsule.SetLocalMatrix(matrix);

		ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
		compound->AddCollision(&capsule);

		matrix.m_posit.m_z += spacing;
	}
}

static void AddStaticMesh(ndDemoEntityManager* const scene, const char* const meshName, ndShapeInstance& sceneInstance, ndDemoEntity* const rootEntity, const ndMatrix& location)
{
	ndMeshEffectNode* const meshEffectNode = LoadFbxMeshEffectNode(meshName);
	ndAssert(meshEffectNode);
	meshEffectNode->m_matrix = location;

	ndDemoEntity* const visualEntity = new ndDemoEntity(scene, meshEffectNode);
	visualEntity->Attach(rootEntity);
	visualEntity->ResetMatrix(location);

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	
	ndInt32 stack = 1;
	ndMatrix matrixBuffer[1024];
	ndMeshEffectNode* entBuffer[1024];
	
	entBuffer[0] = meshEffectNode;
	matrixBuffer[0] = meshEffectNode->m_matrix.Inverse();
	
	while (stack)
	{
		stack--;
		ndMeshEffectNode* const ent = entBuffer[stack];
		ndMatrix matrix(ent->m_matrix * matrixBuffer[stack]);
	
		ndSharedPtr<ndMeshEffect> meshEffect = ent->GetMesh();
		if (*meshEffect)
		{
			ndInt32 vertexStride = meshEffect->GetVertexStrideInByte() / ndInt32 (sizeof(ndFloat64));
			const ndFloat64* const vertexData = meshEffect->GetVertexPool();
	
			ndInt32 mark = meshEffect->IncLRU();
			ndPolyhedra::Iterator iter(*(*meshEffect));
	
			ndVector face[256];
			ndMatrix worldMatrix(ent->m_meshMatrix * matrix);
			for (iter.Begin(); iter; iter++)
			{
				ndEdge* const edge = &(*iter);
				if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
				{
					ndInt32 count = 0;
					ndEdge* ptr = edge;
					do
					{
						ndInt32 i = ptr->m_incidentVertex * vertexStride;
						ndVector point(ndFloat32(vertexData[i + 0]), ndFloat32(vertexData[i + 1]), ndFloat32(vertexData[i + 2]), ndFloat32(1.0f));
						face[count] = worldMatrix.TransformVector(point);
						count++;
						ptr->m_mark = mark;
						ptr = ptr->m_next;
					} while (ptr != edge);
	
					ndInt32 materialIndex = meshEffect->GetFaceMaterial(edge);
					meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);
				}
			}
		}
	
		for (ndMeshEffectNode* child = (ndMeshEffectNode*)ent->GetFirstChild(); child; child = (ndMeshEffectNode*)child->GetNext())
		{
			entBuffer[stack] = child;
			matrixBuffer[stack] = matrix;
			stack++;
		}
	}
	meshBuilder.End(true);
	ndShapeInstance shape(new ndShapeStatic_bvh(meshBuilder));

	ndShapeMaterial material(shape.GetMaterial());
	material.m_data.m_userData = visualEntity;
	shape.SetMaterial(material);
	
	shape.SetLocalMatrix(location);
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->AddCollision(&shape);

	delete meshEffectNode;
}

ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const ndMatrix& location)
{
	ndDemoEntity* const rootEntity = new ndDemoEntity(location, nullptr);
	ndShapeInstance sceneInstance (new ndShapeCompound());
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();

	ndMatrix subShapeLocation(ndGetIdentityMatrix());
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

	scene->AddEntity(rootEntity);
	world->AddBody(body);
	return body;
}