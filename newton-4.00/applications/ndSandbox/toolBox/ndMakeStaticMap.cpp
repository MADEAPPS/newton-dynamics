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
#include "ndPhysicsWorld.h"
#include "ndDemoEntityManager.h"

ndBodyKinematic* BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName)
{
	fbxDemoEntity* const entity = LoadFbxMesh(meshName);
	entity->BuildRenderMeshes(scene);
	scene->AddEntity(entity);
	
	dPolygonSoupBuilder meshBuilder;
	//meshBuilder.Begin();
	//
	//dInt32 stack = 1;
	//fbxDemoEntity* entBuffer[1024];
	//dMatrix matrixBuffer[1024];
	//entBuffer[0] = entity;
	//matrixBuffer[0] = dGetIdentityMatrix();
	//while (stack)
	//{
	//	stack--;
	//	fbxDemoEntity* const ent = entBuffer[stack];
	//	dMatrix matrix (ent->GetCurrentMatrix() * matrixBuffer[stack]);
	//
	//	if (ent->m_fbxMeshEffect)
	//	{
	//		//dInt32 vertexCount = ent->m_fbxMeshEffect->GetVertexCount();
	//		dInt32 vertexStride = ent->m_fbxMeshEffect->GetVertexStrideInByte() / sizeof (dFloat64);
	//		const dFloat64* const vertexData = ent->m_fbxMeshEffect->GetVertexPool();
	//
	//		dInt32 mark = ent->m_fbxMeshEffect->IncLRU();
	//		dPolyhedra::Iterator iter(*ent->m_fbxMeshEffect);
	//		
	//		dVector face[256];
	//
	//		dMatrix worldhMatrix(ent->GetMeshMatrix() * matrix);
	//		for (iter.Begin(); iter; iter++)
	//		{
	//			dEdge* const edge = &(*iter);
	//			if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
	//			{
	//				dInt32 count = 0;
	//				dEdge* ptr = edge;
	//				do
	//				{
	//					dInt32 i = ptr->m_incidentVertex * vertexStride;
	//					dVector point(dFloat32(vertexData[i + 0]), dFloat32(vertexData[i + 1]), dFloat32(vertexData[i + 2]), dFloat32(1.0f));
	//					face[count] = worldhMatrix.TransformVector(point);
	//					count++;
	//					ptr->m_mark = mark;
	//					ptr = ptr->m_next;
	//				} while (ptr != edge);
	//				meshBuilder.AddFace(&face[0].m_x, sizeof(dVector), 3, 31);
	//			}
	//		}
	//	}
	//
	//	for (fbxDemoEntity* child = (fbxDemoEntity*)ent->GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
	//	{
	//		entBuffer[stack] = child;
	//		matrixBuffer[stack] = matrix;
	//		stack++;
	//	}
	//}
	////meshBuilder.End(true);
	//meshBuilder.End(false);
	//
	//ndShapeInstance shape(new ndShapeStaticBVH(meshBuilder));
	//
	//dMatrix matrix(dGetIdentityMatrix());
	//ndBodyDynamic* const body = new ndBodyDynamic();
	//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	//body->SetMatrix(matrix);
	//body->SetCollisionShape(shape);
	//scene->GetWorld()->AddBody(body);
	//
	//entity->CleanIntermediate();
	//return body;
	return nullptr;
}