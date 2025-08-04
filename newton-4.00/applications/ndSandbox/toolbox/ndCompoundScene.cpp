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
#include "ndMeshLoader.h"
#include "ndDemoEntity.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndCompoundScene.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

static void AddStaticMesh(ndDemoEntityManager* const scene, const char* const meshName, ndShapeInstance& sceneInstance, const ndSharedPtr<ndDemoEntity>& rootEntity, const ndMatrix& location)
{
	ndMeshLoader loader;
	ndSharedPtr<ndMesh> meshEffectNode (loader.LoadMesh(meshName));
	ndAssert(meshEffectNode);
	meshEffectNode->m_matrix = location;

	ndSharedPtr<ndDemoEntity> visualEntity (new ndDemoEntity(scene, *meshEffectNode));
	rootEntity->AddChild(visualEntity);
	visualEntity->ResetMatrix(location);

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	ndFixSizeArray<ndMesh*, 1024> entBuffer;
	ndFixSizeArray<ndMatrix, 1024> matrixBuffer;

	entBuffer.PushBack(*meshEffectNode);
	matrixBuffer.PushBack(meshEffectNode->m_matrix.OrthoInverse());

	while (entBuffer.GetCount())
	{
		ndMesh* const ent = entBuffer.Pop();
		const ndMatrix matrix(ent->m_matrix * matrixBuffer.Pop());

		ndSharedPtr<ndMeshEffect> meshEffect = ent->GetMesh();
		if (*meshEffect)
		{
			ndInt32 vertexStride = meshEffect->GetVertexStrideInByte() / ndInt32(sizeof(ndFloat64));
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

		for (ndMesh* child = (ndMesh*)ent->GetFirstChild(); child; child = (ndMesh*)child->GetNext())
		{
			entBuffer.PushBack(child);
			matrixBuffer.PushBack(matrix);
		}
	}
	meshBuilder.End(true);
	ndShapeInstance shape(new ndShapeStatic_bvh(meshBuilder));

	ndShapeMaterial material(shape.GetMaterial());
	material.m_data.m_userData = *visualEntity;
	shape.SetMaterial(material);

	shape.SetLocalMatrix(location);
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->AddCollision(&shape);
}

static void AddSpeedBumpsSubShape(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance, const ndSharedPtr<ndDemoEntity>& rootEntity, const ndMatrix& location, ndInt32 count)
{
	ndShapeInstance capsule(new ndShapeCapsule(0.75f, 0.75f, 10.0f));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("capsule", scene->GetShaderCache(), &capsule, "Concrete_011_COLOR.png", "Concrete_011_COLOR.png", "Concrete_011_COLOR.png", 1.0f, uvMatrix));

	ndFloat32 spacing = 3.0f;
	ndMatrix matrix(location);
	matrix.m_posit.m_y += -0.6f;
	matrix.m_posit.m_z -= (ndFloat32)(count / 2) * spacing;
	ndShapeMaterial material(capsule.GetMaterial());
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndSharedPtr<ndDemoEntity> entity (new ndDemoEntity(matrix));
		rootEntity->AddChild(entity);
		entity->SetMesh(geometry);
		entity->SetName("capsule");

		material.m_data.m_userData = *entity;
		capsule.SetMaterial(material);
		capsule.SetLocalMatrix(matrix);

		ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
		compound->AddCollision(&capsule);

		matrix.m_posit.m_z += spacing;
	}
}

static void AddBoxSubShape(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance, const ndSharedPtr<ndDemoEntity>& rootEntity, const ndMatrix& location)
{
	ndShapeInstance box(new ndShapeBox(0.25f, 4.0f, 0.25f));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.png", "marbleCheckBoard.png", "marbleCheckBoard.png", 1.0f, uvMatrix));

	ndMatrix matrix(location);
	matrix.m_posit.m_y += 1.9f;
	ndSharedPtr<ndDemoEntity> entity (new ndDemoEntity(matrix));
	rootEntity->AddChild(entity);
	entity->SetMesh(geometry);
	entity->SetName("box");

	ndShapeMaterial material(box.GetMaterial());
	material.m_data.m_userData = *entity;
	box.SetMaterial(material);

	box.SetLocalMatrix(matrix);
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->AddCollision(&box);
}

ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const ndMatrix& location)
{
	ndSharedPtr<ndDemoEntity> rootEntity (new ndDemoEntity(location));
	rootEntity->SetName("arena");
	ndShapeInstance sceneInstance(new ndShapeCompound());
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();

	ndMatrix subShapeLocation(ndGetIdentityMatrix());
	AddStaticMesh(scene, "playerarena.fbx", sceneInstance, rootEntity, subShapeLocation);

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

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, rootEntity));
	body->SetMatrix(location);
	body->GetAsBodyDynamic()->SetCollisionShape(sceneInstance);

	// add the bridge
	ndSharedPtr<ndDemoEntity> pivot0 (rootEntity->Find(rootEntity, "pivot1"));
	ndSharedPtr<ndDemoEntity> pivot1 (rootEntity->Find(rootEntity, "pivot0"));
	if (*pivot0 && *pivot0)
	{
		ndMatrix matrix0(pivot0->CalculateGlobalMatrix());
		ndMatrix matrix1(pivot1->CalculateGlobalMatrix());
		ndVector dir(matrix1.m_posit - matrix0.m_posit);
		ndFloat32 lenght = ndSqrt(dir.DotProduct(dir).GetScalar());

		const ndInt32 plankCount = 30;
		ndFloat32 sizex = 10.0f;
		ndFloat32 sizey = 0.25f;
		ndFloat32 sizez = lenght / plankCount;
		ndFloat32 deflection = 0.02f;

		ndMatrix matrix(matrix0);
		matrix.m_posit.m_y -= sizey * 0.5f;
		matrix.m_posit.m_z += sizez * 0.5f;
		ndShapeInstance plankShape(new ndShapeBox(sizex, sizey, sizez + deflection));

		ndFixSizeArray<ndBodyKinematic*, plankCount> array;
		for (ndInt32 i = 0; i < plankCount; ++i)
		{
			ndSharedPtr<ndBody> bodyPart(AddBox(scene, matrix, 20.0f, sizex, sizey, sizez + deflection));
			array.PushBack(bodyPart->GetAsBodyKinematic());
			matrix.m_posit.m_z += sizez;
		}

		for (ndInt32 i = 1; i < plankCount; ++i)
		{
			ndBodyKinematic* body0 = array[i - 1];
			ndBodyKinematic* body1 = array[i];
			ndMatrix linkMatrix(body0->GetMatrix());
			linkMatrix.m_posit = ndVector::m_half * (body0->GetMatrix().m_posit + body1->GetMatrix().m_posit);
			linkMatrix.m_posit.m_y += sizey * 0.5f;
			ndMatrix matrix_0(linkMatrix);
			ndMatrix matrix_1(linkMatrix);
			matrix_0.m_posit.m_z += deflection * 0.5f;
			matrix_1.m_posit.m_z -= deflection * 0.5f;
			ndJointHinge* const hinge = new ndJointHinge(matrix_0, matrix_1, body0, body1);
			hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
			ndSharedPtr<ndJointBilateralConstraint> jointptr(hinge);
			scene->GetWorld()->AddJoint(jointptr);
		}

		{
			ndBodyKinematic* body0 = array[0];
			ndBodyKinematic* body1 = body->GetAsBodyDynamic();
			ndMatrix linkMatrix(body0->GetMatrix());
			linkMatrix.m_posit = body0->GetMatrix().m_posit;
			linkMatrix.m_posit.m_z -= (sizez + deflection) * 0.5f;
			linkMatrix.m_posit.m_y += sizey * 0.5f;
			ndMatrix matrix_0(linkMatrix);
			ndMatrix matrix_1(linkMatrix);
			matrix_0.m_posit.m_z += deflection * 0.5f;
			matrix_1.m_posit.m_z -= deflection * 0.5f;
			ndJointHinge* const hinge = new ndJointHinge(matrix_0, matrix_1, body0, body1);
			hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
			ndSharedPtr<ndJointBilateralConstraint> jointptr(hinge);
			scene->GetWorld()->AddJoint(jointptr);
		}

		{
			ndBodyKinematic* body0 = array[plankCount - 1];
			ndBodyKinematic* body1 = body->GetAsBodyDynamic();
			ndMatrix linkMatrix(body0->GetMatrix());
			linkMatrix.m_posit = body0->GetMatrix().m_posit;
			linkMatrix.m_posit.m_z += (sizez + deflection) * 0.5f;
			linkMatrix.m_posit.m_y += sizey * 0.5f;
			ndMatrix matrix_0(linkMatrix);
			ndMatrix matrix_1(linkMatrix);
			matrix_0.m_posit.m_z += deflection * 0.5f;
			matrix_1.m_posit.m_z -= deflection * 0.5f;
			ndJointHinge* const hinge = new ndJointHinge(matrix_0, matrix_1, body0, body1);
			hinge->SetAsSpringDamper(0.02f, 0.0f, 20.0f);
			ndSharedPtr<ndJointBilateralConstraint> jointptr(hinge);
			scene->GetWorld()->AddJoint(jointptr);
		}
	}

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(body);
	scene->AddEntity(rootEntity);
	return body->GetAsBodyDynamic();
}
