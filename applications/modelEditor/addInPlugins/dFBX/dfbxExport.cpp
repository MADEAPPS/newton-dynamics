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
	static dfbxExport plugin("*.fbx", "Export Autodesk fbx file (.fbx)", "fbx mesh export");
	return &plugin;
}

dExportPlugin* dfbxExport::GetPluginOBJ()
{
	static dfbxExport plugin("*.obj", "Export Wavefront obj file (.obj)", "obj mesh export");
	return &plugin;
}

dExportPlugin* dfbxExport::GetPluginDAE()
{
	static dfbxExport plugin("*.dae", "Export Autodesk Collada file (.dae)", "dae mesh export");
	return &plugin;
}



void dfbxExport::Export (const char* const fileName, dPluginInterface* const interface)
{

	//NewtonWorld* const world = scene->GetNewtonWorld();
	//dAssert (world);

	// Initialize the SDK manager. This object handles memory management.
	FbxManager* const fbxSdk = FbxManager::Create();
	dAssert (fbxSdk);

	int fileFormat = -1;
	if (!stricmp (m_ext, "*.fbx")) {
		// for FBX formats, try sitting ascii format
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
	}


	// Create the IO settings object.
	FbxIOSettings* const ios = FbxIOSettings::Create(fbxSdk, IOSROOT);
	fbxSdk->SetIOSettings(ios);

	// create an exporter
	FbxExporter* const fbxExporter = FbxExporter::Create(fbxSdk, "");

	// Use the first argument as the filename for the importer.
	if (fbxExporter->Initialize(fileName, fileFormat, fbxSdk->GetIOSettings())) { 
		dPluginScene* const ngdScene = interface->GetScene();
		dAssert (ngdScene);

		// rotate scene 90 degree around the y axis
		dMatrix rotateScene (dGetZeroMatrix());
		rotateScene[0][2] = -1.0f;
		rotateScene[1][1] = 1.0f;
		rotateScene[2][0] = 1.0f;
		rotateScene[3][3] = 1.0f;
		ngdScene->BakeTransform (rotateScene);

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

		MeshMap meshMap;
		TextureMap textureMap;
		MaterialMap materialMap;
		TextureIndex textureIndex;
		MaterialIndex materialIndex;
		
		CreateTexture (ngdScene, fbxScene, textureMap, textureIndex);
		CreateMaterials (ngdScene, fbxScene, textureMap, materialMap, materialIndex);
		BuildMeshes (ngdScene, fbxScene, meshMap, textureMap, materialMap, materialIndex, textureIndex);
		LoadNodes (ngdScene, fbxScene, meshMap, materialMap);

		FbxGlobalSettings& settings = fbxScene->GetGlobalSettings();
		settings.SetSystemUnit(FbxSystemUnit(100.0));
		settings.SetAxisSystem(FbxAxisSystem::eMayaYUp);

		// Import the contents of the file into the scene.
		fbxExporter->Export(fbxScene);

		// undo the rotation
		ngdScene->BakeTransform (rotateScene.Transpose());
	}

	fbxSdk->SetIOSettings(NULL);

	// The file is imported, so get rid of the importer.
	fbxExporter->Destroy();

	// destroy the IO settings object.
	ios->Destroy();

	// Destroy the SDK manager and all the other objects it was handling.
	fbxSdk->Destroy();
}


void dfbxExport::LoadNodes (dPluginScene* const scene, FbxScene* const fbxScene, MeshMap& meshMap, MaterialMap& materialMap)
{
	dScene::dTreeNode* const root = scene->GetRootNode();

	for (void* ptr = scene->GetFirstChildLink(root); ptr; ptr = scene->GetNextChildLink(root, ptr) ) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink(ptr);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			LoadNode (scene, fbxScene, fbxScene->GetRootNode(), node, meshMap, materialMap);
		}
	}
}

void dfbxExport::LoadNode (dPluginScene* const ngdScene, FbxScene* const fbxScene, FbxNode* const fbxRoot, dPluginScene::dTreeNode* const node, MeshMap& meshMap, MaterialMap& materialMap)
{
	dSceneNodeInfo* const nodeInfo = (dSceneNodeInfo*)ngdScene->GetInfoFromNode(node);
	FbxNode* const fpxNode = FbxNode::Create(fbxScene, nodeInfo->GetName());
	fbxRoot->AddChild(fpxNode);

	{
		dMatrix matrix (nodeInfo->GetTransform());
		//dAssert (((matrix.m_front * matrix.m_up) % matrix.m_right) > 0.0f);
		dVector scale;
		dMatrix stretchAxis;
		dMatrix transformMatrix; 
		matrix.PolarDecomposition (transformMatrix, scale, stretchAxis);
		dVector euler0;
		dVector euler1;
		transformMatrix.GetEulerAngles(euler0, euler1);
		dVector eulers (euler0.Scale (180.0f / 3.14159265359f));

		dAssert (dAbs (stretchAxis[0][0] - 1.0f)  < 1.0e-6f);
		dAssert (dAbs (stretchAxis[1][1] - 1.0f)  < 1.0e-6f);
		dAssert (dAbs (stretchAxis[2][2] - 1.0f)  < 1.0e-6f);
		fpxNode->LclTranslation.Set(FbxVector4 (transformMatrix.m_posit.m_x, transformMatrix.m_posit.m_y, transformMatrix.m_posit.m_z, 1.0));
		fpxNode->LclRotation.Set(FbxVector4 (eulers.m_x, eulers.m_y, eulers.m_z, 0.0));
		fpxNode->LclScaling.Set(FbxVector4 (scale.m_x, scale.m_y, scale.m_z, 0.0));
	}

	{
		dMatrix matrix (nodeInfo->GetGeometryTransform());
		//dAssert (((matrix.m_front * matrix.m_up) % matrix.m_right) > 0.0f);
		dVector scale;
		dMatrix stretchAxis;
		dMatrix transformMatrix; 
		matrix.PolarDecomposition (transformMatrix, scale, stretchAxis);

		dVector euler0;
		dVector euler1;
		transformMatrix.GetEulerAngles(euler0, euler1);
		dVector eulers (euler0.Scale (180.0f / 3.14159265359f));

		dAssert (dAbs (stretchAxis[0][0] - 1.0f)  < 1.0e-6f);
		dAssert (dAbs (stretchAxis[1][1] - 1.0f)  < 1.0e-6f);
		dAssert (dAbs (stretchAxis[2][2] - 1.0f)  < 1.0e-6f);

		fpxNode->SetGeometricTranslation (FbxNode::eSourcePivot, FbxVector4 (transformMatrix.m_posit.m_x, transformMatrix.m_posit.m_y, transformMatrix.m_posit.m_z, 1.0));
		fpxNode->SetGeometricRotation (FbxNode::eSourcePivot, FbxVector4 (eulers.m_x, eulers.m_y, eulers.m_z, 0.0));
		fpxNode->SetGeometricScaling (FbxNode::eSourcePivot, FbxVector4 (scale.m_x, scale.m_y, scale.m_z, 0.0));
	}


	for (void* ptr = ngdScene->GetFirstChildLink(node); ptr; ptr = ngdScene->GetNextChildLink(node, ptr) ) {
		dScene::dTreeNode* const childMeshNode = ngdScene->GetNodeFromLink(ptr);
		dNodeInfo* const childInfo = ngdScene->GetInfoFromNode(childMeshNode);
		if (childInfo->IsType(dGeometryNodeInfo::GetRttiType())) {
			dAssert(meshMap.Find(childMeshNode));
			FbxMesh* const fbxMesh = meshMap.Find(childMeshNode)->GetInfo();
			fpxNode->SetNodeAttribute(fbxMesh);

			for (void* link = ngdScene->GetFirstChildLink(childMeshNode); link; link = ngdScene->GetNextChildLink(childMeshNode, link)) {
				dScene::dTreeNode* const materialNode = ngdScene->GetNodeFromLink(link);
				dNodeInfo* const info = ngdScene->GetInfoFromNode(materialNode);
				if (info->IsType(dMaterialNodeInfo::GetRttiType())) {
					dAssert (materialMap.Find(materialNode));
					FbxSurfaceMaterial* const fbxMaterial = materialMap.Find(materialNode)->GetInfo();
					fpxNode->AddMaterial(fbxMaterial);
				}
			}
			break;
		}
	}
	
	for (void* ptr = ngdScene->GetFirstChildLink(node); ptr; ptr = ngdScene->GetNextChildLink(node, ptr) ) {
		dScene::dTreeNode* const childNode = ngdScene->GetNodeFromLink(ptr);
		dNodeInfo* const childInfo = ngdScene->GetInfoFromNode(childNode);
		if (childInfo->IsType(dSceneNodeInfo::GetRttiType())) {
			LoadNode (ngdScene, fbxScene, fpxNode, childNode, meshMap, materialMap);
		}
	}
}


void dfbxExport::CreateTexture (dPluginScene* const ngdScene, FbxScene* const fbxScene, TextureMap& textureMap, TextureIndex& textureIndex)
{
	int enumerator = 0;
	dScene::dTreeNode* const cacheNode = ngdScene->FindTextureCacheNode ();
	if (cacheNode) {
		for (void* link = ngdScene->GetFirstChildLink(cacheNode); link; link = ngdScene->GetNextChildLink(cacheNode, link)) {
			dScene::dTreeNode* const textureNode = ngdScene->GetNodeFromLink(link);
			dTextureNodeInfo* const textureInfo = (dTextureNodeInfo*) ngdScene->GetInfoFromNode(textureNode);
			dAssert (textureInfo->IsType(dTextureNodeInfo::GetRttiType()));

			FbxFileTexture* const fbxTexture = FbxFileTexture::Create(fbxScene, "Diffuse Texture");

			fbxTexture->SetFileName(textureInfo->GetPathName()); // Resource file is in current directory.
			fbxTexture->SetTextureUse(FbxTexture::eStandard);
			fbxTexture->SetMappingType(FbxTexture::eUV);
			fbxTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
			fbxTexture->SetSwapUV(false);
			fbxTexture->SetTranslation(0.0, 0.0);
			fbxTexture->SetScale(1.0, 1.0);
			fbxTexture->SetRotation(0.0, 0.0);
			fbxTexture->UVSet.Set(FbxString("DiffuseUV")); // Connect texture to the proper UV

			textureMap.Insert (fbxTexture, textureNode);
			textureIndex.Insert(enumerator, textureInfo->GetId());
			enumerator ++;
		}
	}
}


void dfbxExport::CreateMaterials (dPluginScene* const ngdScene, FbxScene* const fbxScene, const TextureMap& textureMap, MaterialMap& materialMap, MaterialIndex& materialIndex)
{
	int enumerator = 0;
	dScene::dTreeNode* const materilCacheNode = ngdScene->FindGetMaterialCacheNode ();
	if (materilCacheNode) {
		for (void* link = ngdScene->GetFirstChildLink(materilCacheNode); link; link = ngdScene->GetNextChildLink(materilCacheNode, link)) {
			dScene::dTreeNode* const materialNode = ngdScene->GetNodeFromLink(link);

			dMaterialNodeInfo* const materialInfo = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(materialNode);
			dAssert (materialInfo ->IsType(dMaterialNodeInfo::GetRttiType()));

			FbxSurfacePhong* const fbxMaterial = FbxSurfacePhong::Create(fbxScene, materialInfo->GetName());
			
			// Generate primary and secondary colors.
			fbxMaterial->Emissive.Set(FbxDouble3 (0.0, 0.0, 0.0));

			dVector ambient (materialInfo->GetAmbientColor());
			fbxMaterial->Ambient.Set(FbxDouble3(ambient[0], ambient[1], ambient[2]));

			dVector difusse (materialInfo->GetDiffuseColor());
			fbxMaterial->Diffuse.Set(FbxDouble3(difusse[0], difusse[1], difusse[2]));

			fbxMaterial->ShadingModel.Set(FbxString("Phong"));

			fbxMaterial->Shininess.Set (materialInfo->GetShininess() / 100.0);
			dVector specular (materialInfo->GetSpecularColor());
			fbxMaterial->Specular.Set (FbxDouble3(specular[0], specular[1], specular[2]));

			fbxMaterial->TransparencyFactor.Set (1.0 - materialInfo->GetOpacity());

			if (materialInfo->GetDiffuseTextId() != -1) {
				dScene::dTreeNode* const textNode = ngdScene->FindTextureByTextId(materialNode, materialInfo->GetDiffuseTextId());
				if (textNode) {
					_ASSERTE (textNode);
					dAssert (textureMap.Find((textNode)));
					FbxTexture* const fbxTexture = textureMap.Find((textNode))->GetInfo();
					fbxMaterial->Diffuse.ConnectSrcObject(fbxTexture);
				}
			}
	/*
			if (materialInfo->GetAmbientTextId() != -1) {
				dScene::dTreeNode* const textNode = ngdScene->FindTextureByTextId(materialNode, materialInfo->GetAmbientTextId());
				if (textNode) {
					_ASSERTE (textNode);
					dAssert (textureMap.Find((textNode)));
					FbxTexture* const fbxTexture = textureMap.Find((textNode))->GetInfo();
					fbxMaterial->Ambient.ConnectSrcObject(fbxTexture);
				}
			}
	*/

			materialMap.Insert(fbxMaterial, materialNode);
			materialIndex.Insert(enumerator, materialInfo->GetId());
			enumerator ++;
		}
	}
}



void dfbxExport::BuildMeshes (dPluginScene* const ngdScene, FbxScene* const fbxScene, MeshMap& meshMap, const TextureMap& textureMap, const MaterialMap& materialMap, const MaterialIndex& materialIndex, const TextureIndex& textureIndex)
{
	dScene::dTreeNode* const geometryCache = ngdScene->FindGetGeometryCacheNode ();
	if (geometryCache) {
		for (void* link = ngdScene->GetFirstChildLink(geometryCache); link; link = ngdScene->GetNextChildLink(geometryCache, link)) {
			dScene::dTreeNode* const ngdMeshNode = ngdScene->GetNodeFromLink(link);
			dNodeInfo* const info = ngdScene->GetInfoFromNode(ngdMeshNode);
			if (info->IsType(dMeshNodeInfo::GetRttiType())) {
				dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*) info;
				char name[256];
				strcpy (name, meshInfo->GetName());
				char* const ptr = strstr (name, "_mesh");
				if (ptr) {
					ptr[0] = 0;
				}
				FbxMesh* const fbxMesh = FbxMesh::Create(fbxScene, name);
				meshMap.Insert(fbxMesh, ngdMeshNode);

				dMatrix matrix (meshInfo->GetPivotMatrix());
				dAssert (((matrix.m_front.CrossProduct(matrix.m_up)).DotProduct3(matrix.m_right)) > 0.0f);
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
				const dFloat64* const vertex = NewtonMeshGetVertexArray (mesh); 

				fbxMesh->InitControlPoints (vertexCount);
				FbxVector4* const points = fbxMesh->GetControlPoints();
				for (int i = 0; i < vertexCount; i ++) {
					points[i] = FbxVector4(vertex[vertexStride * i + 0], vertex[vertexStride * i + 1], vertex[vertexStride * i + 2], 0.0f);
				}

				int attibuteCount = NewtonMeshGetPointCount(mesh); 
				int attibuteStride = NewtonMeshGetPointStrideInByte(mesh) / sizeof (dFloat64);
				dFloat64* const normal = NewtonMeshGetNormalArray (mesh); 
				dFloat64* const uv0 = NewtonMeshGetUV0Array (mesh); 

				// We want to have one normal for each vertex (or control point),
				FbxGeometryElementNormal* const geometryElementNormal = fbxMesh->CreateElementNormal();
				geometryElementNormal->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
				geometryElementNormal->SetReferenceMode(FbxGeometryElement::eDirect);
				geometryElementNormal->GetIndexArray().SetCount(attibuteCount);

				FbxLayerElementUV* const geometryElementUV = fbxMesh->CreateElementUV("UVChannel_1");
				geometryElementUV->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
				geometryElementUV->SetReferenceMode(FbxGeometryElement::eDirect);
				geometryElementUV->GetIndexArray().SetCount(attibuteCount);

				// Set material mapping.
				FbxGeometryElementMaterial* const materialElement = fbxMesh->CreateElementMaterial();
				materialElement->SetMappingMode(FbxGeometryElement::eByPolygon);
				materialElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

				for (void* face = NewtonMeshGetFirstFace(mesh); face; face = NewtonMeshGetNextFace(mesh, face)) {
					if (!NewtonMeshIsFaceOpen(mesh, face)) {
						int faceVertexIndices[256];
						int facePointIndices[256];
						int indexCount = NewtonMeshGetFaceIndexCount (mesh, face);
						NewtonMeshGetFaceIndices (mesh, face, faceVertexIndices);
						NewtonMeshGetFacePointIndices (mesh, face, facePointIndices);

						int texId = -1;
						int matId = -1;
						int faceId = NewtonMeshGetFaceMaterial (mesh, face);
						MaterialIndex::dTreeNode* const matIndeNode = materialIndex.Find(faceId);
						if (matIndeNode) {
								matId = materialIndex.Find(faceId)->GetInfo();
							dScene::dTreeNode* const materialNode = ngdScene->FindMaterialById(faceId);
							dMaterialNodeInfo* const materialInfo = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(materialNode);
							dCRCTYPE difusseTexture = materialInfo->GetDiffuseTextId();
					
							if (difusseTexture != -1) {
								dAssert (textureIndex.Find(difusseTexture));
								texId = textureIndex.Find(difusseTexture)->GetInfo();
							}
						}
		//matId = -1;
	texId = -1;

						fbxMesh->BeginPolygon(matId, texId, false);
						for (int j = 0; j < indexCount; j ++) {
							dAssert (faceVertexIndices[j] < vertexCount);
							dAssert (facePointIndices[j] < attibuteCount);
							fbxMesh->AddPolygon (faceVertexIndices[j], -1);

							int k = facePointIndices[j];
							geometryElementUV->GetDirectArray().Add(FbxVector2(uv0[attibuteStride * k + 0], uv0[attibuteStride * k + 1]));
							geometryElementNormal->GetDirectArray().Add(FbxVector4(normal[attibuteStride * k + 0], normal[attibuteStride * k + 1], normal[attibuteStride * k + 2], 0.0));
						}
						fbxMesh->EndPolygon();
					}
				}
			}
		}
	}
}
