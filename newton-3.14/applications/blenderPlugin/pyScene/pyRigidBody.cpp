/////////////////////////////////////////////////////////////////////////////
// Name:        pyRigodBody.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#include "StdAfx.h"
#include "pyRigidBody.h"

pyRigidBody::pyRigidBody(pyScene* scene, void* rigidBodyNode)
	:pyBaseNodeInfo<dRigidbodyNodeInfo>(scene, rigidBodyNode)
{
}

pyRigidBody::~pyRigidBody(void)
{
}


void pyRigidBody::SetName (const char* name)
{
	char tmp[256];
	strcpy (tmp, name);
	if (!strstr (name, "_rigidbody")) {
		strcat (tmp, "_rigidbody");
	}

	dRigidbodyNodeInfo* info = GetInfo();
	info->SetName(tmp);
}

void pyRigidBody::SetMass (double mass)
{
	dRigidbodyNodeInfo* info = GetInfo();
}


void pyRigidBody::SetShape (int type)
{
	dScene* scene = m_scene->GetScene();
	dScene::dTreeNode* parentNode = scene->FindParentByType((dScene::dTreeNode*)m_node, dSceneNodeInfo::GetRttiType());
	_ASSERTE (parentNode);
	dSceneNodeInfo* nodeInfo = (dSceneNodeInfo*)scene->GetInfoFromNode(parentNode);

	dScene::dTreeNode* meshNode = scene->FindChildByType(parentNode, dMeshNodeInfo::GetRttiType());
	_ASSERTE (meshNode);
	dMeshNodeInfo* meshInfo = (dMeshNodeInfo*)scene->GetInfoFromNode(meshNode);

	dMatrix matrix (meshInfo->GetPivotMatrix() * nodeInfo->CalculateScaleMatrix());
	NewtonMesh* newtonMesh = meshInfo->GetMesh();

	int vertexCount = NewtonMeshGetVertexCount(newtonMesh);
	int vertexStride = NewtonMeshGetVertexStrideInByte(newtonMesh) / sizeof (dFloat64);
	dFloat64* vertex = NewtonMeshGetVertexArray(newtonMesh);
	dVector* savedVertex = new dVector[vertexCount];

	for (int i = 0; i < vertexCount; i ++) {
		int index = i * vertexStride;
		savedVertex[i] = dVector(dFloat (vertex[index + 0]), dFloat(vertex[index + 1]), dFloat(vertex[index + 2]), dFloat(vertex[index + 3])); 
	}
	matrix.TransformTriplex(vertex, vertexStride * sizeof (dFloat), vertex, vertexStride * sizeof (dFloat), vertexCount);

	
//	dCollisionNodeInfo* shape = NULL;
	dScene::dTreeNode* shapeNode = NULL;
	switch (type)
	{
		case m_box:
		{
			dVector size;
			dMatrix offset;
			NewtonMeshCalculateOOBB(newtonMesh, &matrix[0][0], &size[0], &size[1], &size[2]);

			shapeNode = scene->CreateCollisionBoxNode((dScene::dTreeNode*)m_node);
			dCollisionBoxNodeInfo* info = (dCollisionBoxNodeInfo*) scene->GetInfoFromNode(shapeNode);
			info->SetSize (size.Scale (2.0f));
			break;
		}

		case m_sphere: 
		{
			dVector size;
			dMatrix offset;
			NewtonMeshCalculateOOBB(newtonMesh, &matrix[0][0], &size[0], &size[1], &size[2]);

			shapeNode = scene->CreateCollisionSphereNode((dScene::dTreeNode*)m_node);
			dCollisionSphereNodeInfo* info = (dCollisionSphereNodeInfo*) scene->GetInfoFromNode(shapeNode);
			info->SetRadius(size);
			break;
		}

		case m_cylinder: 
		{
			dVector size;
			dMatrix offset;
			NewtonMeshCalculateOOBB(newtonMesh, &matrix[0][0], &size[0], &size[1], &size[2]);

			shapeNode = scene->CreateCollisionSphereNode((dScene::dTreeNode*)m_node);
			dCollisionSphereNodeInfo* info = (dCollisionSphereNodeInfo*) scene->GetInfoFromNode(shapeNode);
			info->SetRadius(size);
			break;
		}


		case m_convexHull: 
		{
			_ASSERTE (0);
/*
			dVector size;
			dMatrix offset;
			NewtonMeshCalculateOOBB(newtonMesh, &matrix[0][0], &size[0], &size[1], &size[2]);

			shapeNode = scene->CreateCollisionConvexHullNode((dScene::dTreeNode*)m_node);
			dCollisionConvexHullNodeInfo* convexShape = (dCollisionConvexHullNodeInfo*) scene->GetInfoFromNode(shapeNode);

			dMatrix maintInv (matrix.Inverse());
			dCollisionConvexHullNodeInfo* info = (dCollisionConvexHullNodeInfo*) scene->GetInfoFromNode(shapeNode);

			maintInv.TransformTriplex(vertex, vertexStride * sizeof (dFloat), vertex, vertexStride * sizeof (dFloat), vertexCount);
			info->SetFaceSelection (vertexCount, vertex, vertexStride * sizeof (dFloat));
*/
			break;
		}


		case m_cone: 
		{
//			dVector size;
//			dMatrix offset;
//			NewtonMeshCalculateOOBB(newtonMesh, &matrix[0][0], &size[0], &size[1], &size[2]);
//			dScene::dTreeNode* boxNode = scene->CreateCollisionBoxNode((dScene::dTreeNode*)m_node);
//			dCollisionBoxNodeInfo* boxShape = (dCollisionBoxNodeInfo*) scene->GetInfoFromNode(boxNode);
//			boxShape->SetSize (size.Scale (2.0f));
//			shape = boxShape;
			break;
		}

		case m_collisionTree: 
		{
			dVector size;
			dMatrix offset;
			NewtonMeshCalculateOOBB(newtonMesh, &matrix[0][0], &size[0], &size[1], &size[2]);

			shapeNode = scene->CreateCollisionConvexHullNode((dScene::dTreeNode*)m_node);
			//dCollisionConvexHullNodeInfo* convexShape = (dCollisionConvexHullNodeInfo*) scene->GetInfoFromNode(boxNode);
			//boxShape->SetSize (size.Scale (2.0f));
			//shape = boxShape;
			break;
		}


		default:
			_ASSERTE (0);
	}

	dVector inertia;
	dVector centerOfMassAndVolume;
	dCollisionNodeInfo* info = (dCollisionNodeInfo*) scene->GetInfoFromNode(shapeNode);

	info->SetTransform (matrix);
	info->CalculateInertiaGeometry (scene, inertia, centerOfMassAndVolume);

	info->SetInertiaGeometry (inertia); 
	info->SetCenterOfMassAndVolume (centerOfMassAndVolume); 

	for (int i = 0; i < vertexCount; i ++) {
		int index = i * vertexStride;
		vertex[index + 0] = savedVertex[i][0];
		vertex[index + 1] = savedVertex[i][1];
		vertex[index + 2] = savedVertex[i][2];
		vertex[index + 3] = savedVertex[i][3];
	}

	delete[] savedVertex;
}