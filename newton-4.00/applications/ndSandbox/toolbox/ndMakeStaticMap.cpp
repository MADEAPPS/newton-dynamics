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
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoSplinePathMesh.h"

ndBodyKinematic* BuildFloorBox(ndDemoEntityManager* const scene, const ndMatrix& matrix, bool kinematic)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance box(new ndShapeBox(200.0f, 1.0f, 200.f));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 1.0f / 4.0f;
	uvMatrix[1][1] *= 1.0f / 4.0f;
	uvMatrix[2][2] *= 1.0f / 4.0f;

	ndSharedPtr<ndDemoMeshInterface>geometry (new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.png", "marbleCheckBoard.png", "marbleCheckBoard.png", 1.0f, uvMatrix, false));

	ndMatrix location(matrix);
	location.m_posit.m_y -= 0.5f;
	ndSharedPtr<ndDemoEntity>entity (new ndDemoEntity(location));
	entity->SetMesh(geometry);
	entity->SetShadowMode(false);

	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(location);
	body->GetAsBodyDynamic()->SetCollisionShape(box);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body->GetAsBodyDynamic();
}

ndBodyKinematic* BuildFlatPlane(ndDemoEntityManager* const scene, bool optimized, bool kinematic)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector floor[] =
	{
		{ 200.0f, 0.0f,  200.0f, 1.0f },
		{ 200.0f, 0.0f, -200.0f, 1.0f },
		{ -200.0f, 0.0f, -200.0f, 1.0f },
		{ -200.0f, 0.0f,  200.0f, 1.0f },
	};
	ndInt32 index[][3] = { { 0, 1, 2 },{ 0, 2, 3 } };

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	//meshBuilder.LoadPLY("sword.ply");
	//meshBuilder.LoadPLY("static_mesh.ply");
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(ndVector), 31, &index[0][0], 3);
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(ndVector), 31, &index[1][0], 3);
	meshBuilder.End(optimized);

	ndShapeInstance plane(new ndShapeStatic_bvh(meshBuilder));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 1.0f / 50.0f;
	uvMatrix[1][1] *= 1.0f / 50.0f;
	uvMatrix[2][2] *= 1.0f / 50.0f;

	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("box", scene->GetShaderCache(), &plane, "marbleCheckBoard.png", "marbleCheckBoard.png", "marbleCheckBoard.png", 1.0f, uvMatrix));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndSharedPtr<ndDemoEntity>entity(new ndDemoEntity(matrix));
	entity->SetMesh(geometry);
	entity->SetShadowMode(false);

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(plane);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body->GetAsBodyDynamic();
}

ndBodyKinematic* BuildFlatPlane(ndDemoEntityManager* const scene, bool optimized, bool kinematic)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector floor[] =
	{
		{ 200.0f, 0.0f,  200.0f, 1.0f },
		{ 200.0f, 0.0f, -200.0f, 1.0f },
		{ -200.0f, 0.0f, -200.0f, 1.0f },
		{ -200.0f, 0.0f,  200.0f, 1.0f },
	};
	ndInt32 index[][3] = { { 0, 1, 2 },{ 0, 2, 3 } };

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	//meshBuilder.LoadPLY("sword.ply");
	//meshBuilder.LoadPLY("static_mesh.ply");
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(ndVector), 31, &index[0][0], 3);
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(ndVector), 31, &index[1][0], 3);
	meshBuilder.End(optimized);

	ndShapeInstance plane(new ndShapeStatic_bvh(meshBuilder));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 1.0f / 50.0f;
	uvMatrix[1][1] *= 1.0f / 50.0f;
	uvMatrix[2][2] *= 1.0f / 50.0f;

	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("box", scene->GetShaderCache(), &plane, "marbleCheckBoard.png", "marbleCheckBoard.png", "marbleCheckBoard.png", 1.0f, uvMatrix));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndSharedPtr<ndDemoEntity>entity(new ndDemoEntity(matrix));
	entity->SetMesh(geometry);
	entity->SetShadowMode(false);

	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(plane);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body->GetAsBodyDynamic();
}

ndBodyKinematic* BuildGridPlane(ndDemoEntityManager* const scene, ndInt32 grids, ndFloat32 gridSize, ndFloat32 perturbation, bool kinematic)
{
	ndVector origin(-(ndFloat32)grids * gridSize * 0.5f, 0.0f, -(ndFloat32)grids * gridSize * 0.5f, 1.0f);

	ndArray<ndVector> points;
	for (ndInt32 iz = 0; iz <= grids; iz++)
	{
		ndFloat32 z0 = origin.m_z + (ndFloat32)iz * gridSize;
		for (ndInt32 ix = 0; ix <= grids; ix++)
		{
			ndFloat32 x0 = origin.m_x + (ndFloat32)ix * gridSize;
			points.PushBack(ndVector(x0, ndGaussianRandom(0.0f, perturbation), z0, 1.0f));
		}
	}

	ndMeshEffect meshEffect;
	meshEffect.BeginBuild();

	ndMeshEffect::ndMaterial material;
	ndArray<ndMeshEffect::ndMaterial>& materialArray = meshEffect.GetMaterials();
	strcpy(material.m_textureName, "marbleCheckBoard.png");
	materialArray.PushBack(material);

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	ndVector face[16];
	ndFloat32 uvScale = 1.0 / 16.0f;
	ndInt32 materialIndex = 0;
	for (ndInt32 iz = 0; iz < grids; iz++)
	{ 
		for (ndInt32 ix = 0; ix < grids; ix++)
		{
			ndVector p0(points[(ix + 0) * (grids + 1) + iz + 0]);
			ndVector p1(points[(ix + 1) * (grids + 1) + iz + 0]);
			ndVector p2(points[(ix + 1) * (grids + 1) + iz + 1]);
			ndVector p3(points[(ix + 0) * (grids + 1) + iz + 1]);

			meshEffect.BeginBuildFace();
				meshEffect.AddMaterial(materialIndex);
				meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p0.m_x * uvScale, p0.m_z * uvScale);

				meshEffect.AddMaterial(materialIndex);
				meshEffect.AddPoint(p1.m_x, p1.m_y, p1.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p1.m_x * uvScale, p1.m_z * uvScale);

				meshEffect.AddMaterial(materialIndex);
				meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p2.m_x * uvScale, p2.m_z * uvScale);
			meshEffect.EndBuildFace();

			face[0] = p0;
			face[1] = p1;
			face[2] = p2;
			meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);

			meshEffect.BeginBuildFace();
				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p0.m_x * uvScale, p0.m_z * uvScale);

				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p2.m_x * uvScale, p2.m_z * uvScale);

				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p3.m_x, p3.m_y, p3.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p3.m_x * uvScale, p3.m_z * uvScale);
			meshEffect.EndBuildFace();

			face[0] = p0;
			face[1] = p2;
			face[2] = p3;
			meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);
		}
	}
	meshEffect.EndBuild(0.0f);
	meshBuilder.End(false);

	ndShapeInstance plane(new ndShapeStatic_bvh(meshBuilder));
	ndSharedPtr<ndDemoMeshInterface>geometry (new ndDemoMesh("plane", &meshEffect, scene->GetShaderCache()));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndSharedPtr<ndDemoEntity>entity (new ndDemoEntity(matrix));
	entity->SetMesh(geometry);
	entity->SetShadowMode(false);

	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(plane);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body->GetAsBodyDynamic();
}

ndBodyKinematic* BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName, bool optimized, bool kinematic)
{
	ndMeshLoader loader;
	ndSharedPtr<ndMesh> meshEffectNode (loader.LoadMesh(meshName));
	ndAssert(*meshEffectNode);

	ndSharedPtr<ndDemoEntity> visualEntity (new ndDemoEntity(scene, *meshEffectNode));
	visualEntity->SetShadowMode(false);
	scene->AddEntity(visualEntity);
	
	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	
	ndInt32 stack = 1;
	ndMatrix matrixBuffer[1024];
	ndMesh* entBuffer[1024];
	
	entBuffer[0] = *meshEffectNode;
	matrixBuffer[0] = meshEffectNode->m_matrix.OrthoInverse();
	
	while (stack)
	{
		stack--;
		ndMesh* const ent = entBuffer[stack];
		ndMatrix matrix (ent->m_matrix * matrixBuffer[stack]);
	
		ndSharedPtr<ndMeshEffect> meshEffect = ent->GetMesh();
		if (*meshEffect)
		{
			ndInt32 vertexStride = meshEffect->GetVertexStrideInByte() / ndInt32(sizeof (ndFloat64));
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
	
		for (ndMesh* child = ent->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			matrixBuffer[stack] = matrix;
			stack++;
		}
	}
	meshBuilder.End(optimized);
	ndShapeInstance shape(new ndShapeStatic_bvh(meshBuilder));
	
	ndMatrix matrix(visualEntity->GetCurrentMatrix());
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, visualEntity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	
	scene->GetWorld()->AddBody(body);
	return body->GetAsBodyDynamic();
}

static ndBodyKinematic* CreateBody(ndDemoEntityManager* const scene, const ndShapeInstance& shape, const ndMatrix& location, ndFloat32 mass)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	const char* const textName = "wood_1.png";
	ndMatrix matrix(FindFloor(*world, location, shape, 200.0f));
	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("shape", scene->GetShaderCache(), &shape, textName, textName, textName));

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	ndSharedPtr<ndDemoEntity>entity(new ndDemoEntity(matrix));
	entity->SetMesh(mesh);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body->GetAsBodyDynamic();
}

ndBodyKinematic* BuildPlayArena(ndDemoEntityManager* const scene, bool kinematic)
{
	ndMeshLoader loader;
	ndSharedPtr<ndMesh>meshEffectNode (loader.LoadMesh("playerarena.fbx"));
	ndSharedPtr<ndDemoEntity> entity (new ndDemoEntity(scene, *meshEffectNode));
	entity->SetShadowMode(false);
	scene->AddEntity(entity);

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	ndFixSizeArray<ndMesh*, 1024> entBuffer;
	ndFixSizeArray<ndMatrix, 1024> matrixBuffer;

	entBuffer.PushBack(*meshEffectNode);
	matrixBuffer.PushBack(meshEffectNode->m_matrix.OrthoInverse());

	while (entBuffer.GetCount())
	{
		ndMesh* ent = entBuffer.Pop();
		ndMatrix matrix(ent->m_matrix * matrixBuffer.Pop());

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

		for (ndMesh* child = ent->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer.PushBack(child);
			matrixBuffer.PushBack(matrix);
		}
	}
	meshBuilder.End(true);
	ndShapeInstance shape(new ndShapeStatic_bvh(meshBuilder));
	ndMatrix matrix(entity->GetCurrentMatrix());

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	scene->GetWorld()->AddBody(body);

	ndSharedPtr<ndDemoEntity> pivot0 (entity->Find(entity, "pivot1"));
	ndSharedPtr<ndDemoEntity> pivot1 (entity->Find(entity, "pivot0"));
	if (*pivot0 && *pivot1)
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

		matrix = matrix0;
		matrix.m_posit.m_y -= sizey * 0.5f;
		matrix.m_posit.m_z += sizez * 0.5f;
		ndShapeInstance plankShape(new ndShapeBox(sizex, sizey, sizez + deflection));

		ndFixSizeArray<ndBodyKinematic*, plankCount> array;
		for (ndInt32 i = 0; i < plankCount; ++i)
		{
			array.PushBack(CreateBody(scene, plankShape, matrix, 20.0f));
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
			ndBodyKinematic* body1 = body->GetAsBodyKinematic();
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
			ndBodyKinematic* body1 = body->GetAsBodyKinematic();
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

	return body->GetAsBodyKinematic();
}