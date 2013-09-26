/////////////////////////////////////////////////////////////////////////////
// Name:        NewtonImport.cpp
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
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

#include "stdafx.h"
#include "NewtonImport.h"

#include <ctype.h>


NewtonImport::NewtonImport()
	:dImportPlugin()
{
}

NewtonImport::~NewtonImport()
{
}


NewtonImport* NewtonImport::GetPlugin()
{
	static NewtonImport gImporter;
	return &gImporter;
}


bool NewtonImport::Import (const char* const fileName, dPluginInterface* const interface)
{
_ASSERTE (0);
return true;
	FILE* const file = fopen (fileName, "rb");
	if (file) {
		// save the state of the scene 
		interface->Push (new dUndoCurrentScene(interface));

		NewtonWorld* const world = NewtonCreate();
		NewtonWorldSetUserData(world, interface);
		NewtonDeserializeBodyArray(world, BodyDeserialization, DeserializeFile, file);
		BuildSceneFromWorld(world);
		NewtonDestroy(world);		
		fclose (file);
	}
	return true;
}

void NewtonImport::DeserializeFile (void* const serializeHandle, void* const buffer, int size)
{
	// check that each chunk is a multiple of 4 bytes, this is useful for easy little to big Indian conversion
	_ASSERTE ((size & 0x03) == 0);
	fread (buffer, size, 1, (FILE*) serializeHandle);
}



void NewtonImport::BodyDeserialization (NewtonBody* const body, NewtonDeserializeCallback deserializecallback, void* const serializeHandle)
{
#if 0
	//for nwo do not do anything at all;
/*
	int size;
	char bodyIndentification[256];

	deserializecallback (serializeHandle, &size, sizeof (size));
	deserializecallback (serializeHandle, bodyIndentification, size);

	// get the world and the scene form the world user data
	NewtonWorld* const world = NewtonBodyGetWorld(body);

	dScene* const scene = (dScene*)NewtonWorldGetUserData(world);

	// crate a scene node for this body
	dScene::dTreeNode* const scene = scene->CreateSceneNode(scene->GetRootNode());

	// here we attach a visual object to the entity, 
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);

	DemoEntity* const entity = new DemoEntity(matrix, NULL);
	scene->Append (entity);

	NewtonBodySetUserData (body, entity);
	NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);
	NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);


	//for visual mesh we will collision mesh and convert it to a visual mesh using NewtonMesh 
	NewtonCollision* const collision = NewtonBodyGetCollision(body);
	int collisionID = NewtonCollisionGetType(collision) ;

	DemoMesh* mesh = NULL;
	switch (collisionID) 
	{
	case SERIALIZE_ID_HEIGHTFIELD:
		{
			NewtonCollisionInfoRecord info;
			NewtonCollisionGetInfo(collision, &info);

			const NewtonHeightFieldCollisionParam& heighfield = info.m_heightField;
			mesh = new DemoMesh ("terrain", heighfield.m_elevation, heighfield.m_width, heighfield.m_horizonalScale, 1.0f/16.0f, 128); 
			break;
		}

	default:
		mesh = new DemoMesh("cylinder_1", collision, NULL, NULL, NULL);
		break;
	}

	entity->SetMesh(mesh);
	mesh->Release();
*/


	// search for all collision mesh and create make a dictionary
//	dTree<dSceneNodeCollisionPair, NewtonCollision*> dictionary;

//	int count = 0;
//	dScene::dTreeNode* const materialNode = CreateMaterialNode (GetRootNode(), 0);
//	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
	NewtonCollision* const collision = NewtonBodyGetCollision(body);

//		dTree<dSceneNodeCollisionPair, NewtonCollision*>::dTreeNode* node = dictionary.Find(collision);
//		if (!node) {
//		char meshName[256];
//		sprintf (meshName, "mesh_%d", count);
//		count ++;
//		NewtonMesh* const mesh = visualContext->CreateVisualMesh(body, meshName, sizeof (meshName));
	NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);
	dScene::dTreeNode* const meshNode = scene->CreateMeshNode(scene->GetRootNode());
/*
			AddReference(meshNode, materialNode);

			dMeshNodeInfo* const info = (dMeshNodeInfo*)GetInfoFromNode(meshNode);
			info->ReplaceMesh (mesh);
			info->SetName(meshName);

			NewtonCollisionInfoRecord collsionRecord;
			NewtonCollisionGetInfo(collision, &collsionRecord);

			// extract the offset matrix form the collision
			dMatrix& offsetMatrix = *((dMatrix*)&collsionRecord.m_offsetMatrix[0][0]);
			info->BakeTransform (offsetMatrix.Inverse());
			info->SetPivotMatrix(offsetMatrix * info->GetPivotMatrix());

			dScene::dTreeNode* const collisionNode = CreateCollisionFromNewtonCollision(GetRootNode(), collision);

			dSceneNodeCollisionPair pair;
			pair.m_mesh = meshNode;
			pair.m_collision = collisionNode;

			node = dictionary.Insert(pair, collision);
		} 

		// add a visual mesh
		dSceneNodeCollisionPair& info = node->GetInfo();
		dScene::dTreeNode* const sceneNode = CreateSceneNode(GetRootNode());
		dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*) GetInfoFromNode(sceneNode);
		dMatrix matrix;
		NewtonBodyGetMatrix(body, &matrix[0][0]);
		sceneInfo->SetTransform(matrix);
		AddReference(sceneNode, info.m_mesh);


		// add a rigid body
		dScene::dTreeNode* const sceneBody = CreateRigidbodyNode(sceneNode);
		AddReference(sceneBody, info.m_collision);

		dRigidbodyNodeInfo* const bodyInfo = (dRigidbodyNodeInfo*) GetInfoFromNode(sceneBody);

		dVector com;
		NewtonBodyGetCentreOfMass(body, &com[0]);
		bodyInfo->SetCenterOfMass(com);

		dVector massMatrix;
		NewtonBodyGetMassMatrix(body, &massMatrix.m_w, &massMatrix.m_x, &massMatrix.m_y, &massMatrix.m_z);
		bodyInfo->SetMassMatrix(massMatrix);

		dVector veloc;
		NewtonBodyGetVelocity(body, &veloc[0]);
		bodyInfo->SetVelocity(veloc);

		dVector omega;
		NewtonBodyGetOmega(body, &omega[0]);
		bodyInfo->SetOmega(omega);

		dVariable* var = bodyInfo->CreateVariable ("rigidBodyType");
		var->SetValue("default gravity");
*/
//	}

//	void* nextPtr = NULL;
//	for (void* ptr = GetFirstChild (GetRootNode()); ptr; ptr = nextPtr) {
//		nextPtr = GetNextChild(GetRootNode(), ptr);
//		dScene::dTreeNode* const node = GetNodeFromLink(ptr);
//		dNodeInfo* const info = GetInfoFromNode(node);
//		if ((info->IsType(dMeshNodeInfo::GetRttiType())) || (info->IsType(dCollisionNodeInfo::GetRttiType()))) {
//			RemoveReference(node, GetRootNode());	
//		}
//	}
//	RemoveReference(materialNode, GetRootNode());	
#endif
}




void NewtonImport::BuildSceneFromWorld(NewtonWorld* const world)
{
	dPluginInterface* const interface = (dPluginInterface*)NewtonWorldGetUserData(world);
	dPluginScene* const scene = interface->GetScene();
	dScene::dTreeNode* const root = scene->GetRootNode();

	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		dScene::dTreeNode* const modelNode = scene->CreateModelNode(root);
		dScene::dTreeNode* const rigiBodyNode = scene->CreateRigidbodyNode(modelNode);
		dScene::dTreeNode* const collisionNode = scene->CreateCollisionFromNewtonCollision(rigiBodyNode, collision);
		collisionNode;

	}
}