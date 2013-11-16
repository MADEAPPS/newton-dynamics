/////////////////////////////////////////////////////////////////////////////
// Name:        NewtonMeshEffectExport.cpp
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

#include "dfbxStdafx.h"
#include "dfbxImport.h"


dImportPlugin* dfbxImport::GetPlugin()
{
	static dfbxImport plugin;
	return &plugin;
}

bool dfbxImport::Import (const char* const fileName, dPluginInterface* const interface)
{
	bool ret = false;

	dScene* const scene = interface->GetScene();
	dAssert (scene);

	NewtonWorld* const world = scene->GetNewtonWorld();
	dAssert (world);

	// Initialize the SDK manager. This object handles memory management.
	FbxManager* const fbxSdk = FbxManager::Create();
	dAssert (fbxSdk);

	// Create the IO settings object.
	FbxIOSettings* const ios = FbxIOSettings::Create(fbxSdk, IOSROOT);
	fbxSdk->SetIOSettings(ios);

	// Create an importer using the SDK manager.
	FbxImporter* const fbxImporter = FbxImporter::Create(fbxSdk, "");

	// Use the first argument as the filename for the importer.
	if(fbxImporter->Initialize(fileName, -1, fbxSdk->GetIOSettings())) { 
		ret = true;
		// Create a new scene so that it can be populated by the imported file.
		FbxScene* const fbxScene = FbxScene::Create(fbxSdk,"myScene");

		// Import the contents of the file into the scene.
		fbxImporter->Import(fbxScene);

//		NewtonMeshFixTJoints (mesh);
		dPluginScene* const asset = new dPluginScene(world);
/*
		dString name (GetNameFromPath(fileName));
		name = name.SubString (0, name.Find ('.'));
		dPluginScene::dTreeNode* const sceneNode = asset->CreateSceneNode (asset->GetRoot());
		dSceneModelInfo* const sceneNodeInfo = (dSceneModelInfo*) asset->GetInfoFromNode(sceneNode);
		sceneNodeInfo->SetName(name.GetStr());	
		dPluginScene::dTreeNode* const meshNode = asset->CreateMeshNode(sceneNode);
		dMeshNodeInfo* const instance = (dMeshNodeInfo*) asset->GetInfoFromNode(meshNode);
		instance->SetName ((name + "_mesh").GetStr());
		instance->ReplaceMesh (mesh);
*/
		PopulateScene (fbxScene, asset);

		interface->MergeScene (asset);
		asset->Release();

		// destroy the scene
		fbxScene->Destroy();
	}

	// The file is imported, so get rid of the importer.
	fbxImporter->Destroy();

	// Destroy the SDK manager and all the other objects it was handling.
	fbxSdk->Destroy();
	return ret;
}


void dfbxImport::PopulateScene (const FbxScene* const fbxScene, dPluginScene* const ngdScene)
{

	LoadHiearchy (fbxScene, ngdScene);
/*
	dList <ImportStackData> nodeStack; 
	
	// Print the nodes of the scene and their attributes recursively.
	FbxNode* const rootNode = fbxScene->GetRootNode();
	if(rootNode) {
		int count = rootNode->GetChildCount();
		for(int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData (GetIdentityMatrix(), rootNode->GetChild(count - i - 1), ngdScene->GetRoot()));
		}
	}

	while (nodeStack.GetCount()) {

		ImportStackData data (nodeStack.GetLast()->GetInfo());
		nodeStack.Remove(nodeStack.GetLast());

		FbxAMatrix fpxNoceMatrix (data.m_fbxNode->EvaluateLocalTransform());

		dMatrix matrix (fpxNoceMatrix);
//		FbxDouble3 translation = data.m_fbxNode->LclTranslation.Get();
//		FbxDouble3 rotation = data.m_fbxNode->LclRotation.Get();
//		FbxDouble3 scaling = data.m_fbxNode->LclScaling.Get();

		FbxNodeAttribute* const attribute = data.m_fbxNode->GetNodeAttribute();
		dPluginScene::dTreeNode* parentNode = data.m_parentNode;
		if (attribute) {
			FbxNodeAttribute::EType attributeType = attribute->GetAttributeType();

			switch (attributeType)
			{
				case FbxNodeAttribute::eMesh: 
				{
					parentNode = ImportMeshNode (data.m_fbxNode, ngdScene, parentNode);
					break;
				}

				case FbxNodeAttribute::eNull:
				case FbxNodeAttribute::eMarker:
				case FbxNodeAttribute::eSkeleton: 
				case FbxNodeAttribute::eNurbs: 
				case FbxNodeAttribute::ePatch:
				case FbxNodeAttribute::eCamera: 
				case FbxNodeAttribute::eCameraStereo:
				case FbxNodeAttribute::eCameraSwitcher:
				case FbxNodeAttribute::eLight:
				case FbxNodeAttribute::eOpticalReference:
				case FbxNodeAttribute::eOpticalMarker:
				case FbxNodeAttribute::eNurbsCurve:
				case FbxNodeAttribute::eTrimNurbsSurface:
				case FbxNodeAttribute::eBoundary:
				case FbxNodeAttribute::eNurbsSurface:
				case FbxNodeAttribute::eShape:
				case FbxNodeAttribute::eLODGroup:
				case FbxNodeAttribute::eSubDiv:
				case FbxNodeAttribute::eCachedEffect:
				case FbxNodeAttribute::eLine:
				case FbxNodeAttribute::eUnknown:
				default:
					dAssert(0);
					break;

			}   
		} else {
			dAssert(0);
		}
		
		int count = rootNode->GetChildCount();
		for(int i = 0; i < count; i++) {
			dAssert (0);
//			parentNode.Append(parent);
//			nodeStack.Append(rootNode->GetChild(count - i - 1));
		}
	}
*/
}





dPluginScene::dTreeNode* dfbxImport::ImportMeshNode (const FbxNode* const fbxMeshNode, dPluginScene* const ngdScene, dPluginScene::dTreeNode* const rootNode)
{

	return NULL;
}


void dfbxImport::LoadHiearchy (const FbxScene* const fbxScene, dPluginScene* const ngdScene)
{
	dList <ImportStackData> nodeStack; 

	// Print the nodes of the scene and their attributes recursively.
	FbxNode* const rootNode = fbxScene->GetRootNode();
	if(rootNode) {
		int count = rootNode->GetChildCount();
		for(int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData (GetIdentityMatrix(), rootNode->GetChild(count - i - 1), ngdScene->GetRoot()));
		}
	}

	while (nodeStack.GetCount()) {
		ImportStackData data (nodeStack.GetLast()->GetInfo());
		nodeStack.Remove(nodeStack.GetLast());

		FbxAMatrix fpxNoceMatrix (data.m_fbxNode->EvaluateLocalTransform());
		
		dPluginScene::dTreeNode* const node = ngdScene->CreateSceneNode (data.m_parentNode);

		dMatrix matrix (fpxNoceMatrix);
		dSceneNodeInfo* const info = (dSceneNodeInfo*) ngdScene->GetInfoFromNode (node);

		info->SetName(data.m_fbxNode->GetName());
		info->SetTransform(matrix);

		int count = data.m_fbxNode->GetChildCount();
		for(int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData (matrix, data.m_fbxNode->GetChild(count - i - 1), node));	
		}
	}
}
