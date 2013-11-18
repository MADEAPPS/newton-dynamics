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
#include "dfbxExport.h"



dExportPlugin* dfbxExport::GetPluginFBX()
{
	static dfbxExport plugin("*.fbx", "Export Autodesk fbx file", "fbx mesh export");
	return &plugin;
}

dExportPlugin* dfbxExport::GetPluginOBJ()
{
	static dfbxExport plugin("*.obj", "Export Wavefront obj file", "obj mesh export");
	return &plugin;
}

dExportPlugin* dfbxExport::GetPluginDAE()
{
	static dfbxExport plugin("*.dae", "Export Autodesk Collada file", "dae mesh export");
	return &plugin;
}



void dfbxExport::Export (const char* const fileName, dPluginInterface* const interface)
{
	dPluginScene* const scene = interface->GetScene();
	dAssert (scene);

	//NewtonWorld* const world = scene->GetNewtonWorld();
	//dAssert (world);

	// Initialize the SDK manager. This object handles memory management.
	FbxManager* const fbxSdk = FbxManager::Create();
	dAssert (fbxSdk);

	// for FBX formats, try sitting ascii format
	int fileFormat = -1;
	int formatsCount = fbxSdk->GetIOPluginRegistry()->GetWriterFormatCount();
	for (int i = 0; i < formatsCount; i++)
	{
		if (fbxSdk->GetIOPluginRegistry()->WriterIsFBX(i)) {
			const FbxString& lDesc = fbxSdk->GetIOPluginRegistry()->GetWriterFormatDescription(i);
			if (lDesc.Find("ascii") >= 0)
			{
				fileFormat = i;
				break;
			}
		}
	}


	// Create the IO settings object.
	FbxIOSettings* const ios = FbxIOSettings::Create(fbxSdk, IOSROOT);
	fbxSdk->SetIOSettings(ios);

	// create an exporter
	FbxExporter* const fbxExporter = FbxExporter::Create(fbxSdk, "");

	// Use the first argument as the filename for the importer.
	if (fbxExporter->Initialize(fileName, fileFormat, fbxSdk->GetIOSettings())) { 

		// Create a new scene so that it can be populated by the imported file.
		FbxScene* const fbxScene = FbxScene::Create(fbxSdk,"myScene");

		// set the scene information
		FbxDocumentInfo* const sceneInfo = FbxDocumentInfo::Create(fbxSdk, "SceneInfo");
		sceneInfo->mTitle = "Example scene";
		sceneInfo->mSubject = "";
		sceneInfo->mAuthor = "Newton Game Dynamic, model editor";
		sceneInfo->mRevision = "revision 1.0";
		sceneInfo->mKeywords = "";
		sceneInfo->mComment = "converted from NGD format, for more info go to: http://newtondynamics.com";
		fbxScene->SetSceneInfo(sceneInfo);

		dTree<FbxMesh*, dPluginScene::dTreeNode*> meshMap;
		BuildMeshes (scene, fbxScene, meshMap);
		LoadNodes (scene, fbxScene, meshMap);

//		const  FbxSystemUnit::ConversionOptions lConversionOptions = {true, true, true, true, true, true};
//		FbxSystemUnit::Inch.ConvertScene(fbxScene, lConversionOptions);
//		FbxAxisSystem::Max.ConvertScene(fbxScene);

		// Import the contents of the file into the scene.
		fbxExporter->Export(fbxScene);
	}

	fbxSdk->SetIOSettings(NULL);

	// The file is imported, so get rid of the importer.
	fbxExporter->Destroy();

	// destroy the IO settings object.
	ios->Destroy();

	// Destroy the SDK manager and all the other objects it was handling.
	fbxSdk->Destroy();
}


void dfbxExport::LoadNodes (dPluginScene* const scene, FbxScene* const fbxScene, dTree<FbxMesh*, dPluginScene::dTreeNode*>& meshMap)
{
	dScene::dTreeNode* const root = scene->GetRootNode();

	for (void* ptr = scene->GetFirstChildLink(root); ptr; ptr = scene->GetNextChildLink(root, ptr) ) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink(ptr);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			LoadNode (scene, fbxScene, fbxScene->GetRootNode(), node, meshMap);
		}
	}
}

void dfbxExport::LoadNode (dPluginScene* const scene, FbxScene* const fbxScene, FbxNode* const fbxRoot, dPluginScene::dTreeNode* const node, dTree<FbxMesh*, dPluginScene::dTreeNode*>& meshMap)
{
	dSceneNodeInfo* const nodeInfo = (dSceneNodeInfo*)scene->GetInfoFromNode(node);
	FbxNode* const fpxNode = FbxNode::Create(fbxScene, nodeInfo->GetName());
	fbxRoot->AddChild(fpxNode);

	dMatrix matrix (nodeInfo->GetTransform());
	FbxMatrix fbxMatrix;
	double* const data = fbxMatrix;
	for (int i = 0; i < 4; i ++) {
		for (int j = 0; j < 4; j ++) {
			data[i * 4 + j] = matrix[i][j];
		}
	}
	FbxVector4 translation;
	FbxQuaternion rotation;
	FbxVector4 shearing;
	FbxVector4 scaling;
	double sign;
	fbxMatrix.GetElements(translation, rotation, shearing, scaling, sign);

	fpxNode->LclTranslation.Set(translation);
	fpxNode->LclRotation.Set(rotation.DecomposeSphericalXYZ());
	fpxNode->LclScaling.Set(scaling);

	for (void* ptr = scene->GetFirstChildLink(node); ptr; ptr = scene->GetNextChildLink(node, ptr) ) {
		dScene::dTreeNode* const childNode = scene->GetNodeFromLink(ptr);
		dNodeInfo* const childInfo = scene->GetInfoFromNode(childNode);
		if (childInfo->IsType(dGeometryNodeInfo::GetRttiType())) {
			dAssert(meshMap.Find(childNode));
			FbxMesh* const fbxMesh = meshMap.Find(childNode)->GetInfo();
			fpxNode->SetNodeAttribute(fbxMesh);
			break;
		}
	}
	
	for (void* ptr = scene->GetFirstChildLink(node); ptr; ptr = scene->GetNextChildLink(node, ptr) ) {
		dScene::dTreeNode* const childNode = scene->GetNodeFromLink(ptr);
		dNodeInfo* const childInfo = scene->GetInfoFromNode(childNode);
		if (childInfo->IsType(dSceneNodeInfo::GetRttiType())) {
			LoadNode (scene, fbxScene, fpxNode, childNode, meshMap);
		}
	}
}


void dfbxExport::BuildMeshes (dPluginScene* const ngdScene, FbxScene* const fbxScene, dTree<FbxMesh*, dPluginScene::dTreeNode*>& meshMap)
{
	dScene::dTreeNode* const geometryCache = ngdScene->FindGetGeometryCacheNode ();
	for (void* link = ngdScene->GetFirstChildLink(geometryCache); link; link = ngdScene->GetNextChildLink(geometryCache, link)) {
		dScene::dTreeNode* const node = ngdScene->GetNodeFromLink(link);
		dNodeInfo* const info = ngdScene->GetInfoFromNode(node);
		if (info->IsType(dMeshNodeInfo::GetRttiType())) {
			dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*) info;
			char name[256];
			strcpy (name, meshInfo->GetName());
			char* const ptr = strstr (name, "_mesh");
			if (ptr) {
				ptr[0] = 0;
			}
			FbxMesh* const fbxMesh = FbxMesh::Create(fbxScene, name);
			meshMap.Insert(fbxMesh, node);

			dMatrix matrix (meshInfo->GetPivotMatrix());
			FbxAMatrix fbxMatrix;
			double* const data = fbxMatrix;
			for (int i = 0; i < 4; i ++) {
				for (int j = 0; j < 4; j ++) {
					data[i * 4 + j] = matrix[i][j];
				}
			}
			fbxMesh->SetPivot(fbxMatrix);

			NewtonMesh* const mesh = meshInfo->GetMesh();
			int vertexCount = NewtonMeshGetVertexCount(mesh);
			int vertexStride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat64);
			dFloat64* const vertex = NewtonMeshGetVertexArray (mesh); 

			fbxMesh->InitControlPoints (vertexCount);
			FbxVector4* const points = fbxMesh->GetControlPoints();
			for (int i = 0; i < vertexCount; i ++) {
				points[i] = FbxVector4(vertex[vertexStride * i + 0], vertex[vertexStride * i + 1], vertex[vertexStride * i + 2], 0.0f);
			}

			// We want to have one normal for each vertex (or control point),
			FbxGeometryElementNormal* const geometryElementNormal = fbxMesh->CreateElementNormal();
			geometryElementNormal->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
			geometryElementNormal->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

			int attibuteCount = NewtonMeshGetPointCount(mesh); 
			int attibuteStride = NewtonMeshGetPointStrideInByte(mesh) / sizeof (dFloat64);
			dFloat64* const normal = NewtonMeshGetNormalArray (mesh); 

			for (int i = 0; i < attibuteCount; i ++) {
				geometryElementNormal->GetDirectArray().Add(FbxVector4(normal[attibuteStride * i + 0], normal[attibuteStride * i + 1], normal[attibuteStride * i + 2], 0.0));
			}

			for (void* face = NewtonMeshGetFirstFace(mesh); face; face = NewtonMeshGetNextFace(mesh, face)) {
				if (!NewtonMeshIsFaceOpen(mesh, face)) {
					int faceIndices[256];
					int indexCount = NewtonMeshGetFaceIndexCount (mesh, face);
					NewtonMeshGetFaceIndices (mesh, face, faceIndices);

					//int matId = NewtonMeshGetFaceMaterial (mesh, face);
					//MaterialProxi material;
					//material.m_mtl = 0;
					//material.m_matID = 0;
					//MaterialCache::dTreeNode* const materialNode = materialCache.Find(matId);
					//if (materialNode) {
					//	material = materialNode->GetInfo();
					//}

					fbxMesh->BeginPolygon(-1, -1, false);
					for (int j = 0; j < indexCount; j ++) {
						fbxMesh->AddPolygon (faceIndices[j], -1);
					}
					fbxMesh->EndPolygon();
				}
			}
		}
	}
}
