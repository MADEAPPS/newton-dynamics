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

#define DEFUALT_MATERIAL_ID -1

dImportPlugin* dfbxImport::GetPluginFBX()
{
	static dfbxImport plugin("*.fbx", "Import Autodesk fbx file", "fbx mesh import");
	return &plugin;
}

dImportPlugin* dfbxImport::GetPluginOBJ()
{
	static dfbxImport plugin("*.obj", "Import Wavefront obj file", "obj mesh import");
	return &plugin;
}

dImportPlugin* dfbxImport::GetPluginDAE()
{
	static dfbxImport plugin("*.dae", "Import Autodesk Collada file", "dae mesh import");
	return &plugin;
}


bool dfbxImport::Import (const char* const fileName, dPluginInterface* const interface)
{
	bool ret = false;

	dPluginScene* const scene = interface->GetScene();
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

		// Convert the scene to meters using the defined options.
		FbxGlobalSettings& settings = fbxScene->GetGlobalSettings();

		dMatrix convertMatrix (GetIdentityMatrix());

		const FbxSystemUnit& systemUnit = settings.GetSystemUnit();
		convertMatrix[0][0] = dFloat (systemUnit.GetScaleFactor() / 100.0f);
		convertMatrix[1][1] = dFloat (systemUnit.GetScaleFactor() / 100.0f);
		convertMatrix[2][2] = dFloat (systemUnit.GetScaleFactor() / 100.0f);

		int sign;
		dVector upVector (0.0f, 1.0f, 0.0f, 0.0f);
		dVector frontVector (1.0f, 0.0f, 0.0f, 0.0f);
		FbxAxisSystem axisSystem = settings.GetAxisSystem();
		if (axisSystem.GetUpVector(sign) == FbxAxisSystem::eXAxis) {
			dAssert (0);
		} else if (axisSystem.GetUpVector(sign) == FbxAxisSystem::eYAxis) {
			upVector = dVector (0.0f, 1.0f * sign, 0.0f, 0.0f);
			if (axisSystem.GetFrontVector(sign) == FbxAxisSystem::eParityEven) {
				frontVector = dVector (1.0f * sign, 0.0f, 0.0f, 0.0f);
			} else {
				frontVector = dVector (0.0f, 0.0f, 1.0f * sign, 0.0f);
			}
		} else {
			upVector = dVector (1.0f * sign, 0.0f, 0.0f, 0.0f);
			if (axisSystem.GetFrontVector(sign) == FbxAxisSystem::eParityEven) {
				dAssert (0);
				frontVector = dVector (1.0f * sign, 0.0f, 0.0f, 0.0f);
			} else {
				frontVector = dVector (0.0f, 0.0f, -1.0f * sign, 0.0f);
			}
		}

		dMatrix axisMatrix (GetIdentityMatrix());
		axisMatrix.m_front = frontVector;
		axisMatrix.m_up = upVector;
		axisMatrix.m_right = frontVector * upVector;
		convertMatrix = axisMatrix * convertMatrix;

//		NewtonMeshFixTJoints (mesh);
		dPluginScene* const asset = new dPluginScene(world);
		PopulateScene (fbxScene, asset);

		asset->RemoveUnusedMaterials();
		asset->BakeTransform (convertMatrix);

		interface->MergeScene (asset);
		asset->Release();

		// destroy the scene
		fbxScene->Destroy();
	}

	fbxSdk->SetIOSettings(NULL);

	// The file is imported, so get rid of the importer.
	fbxImporter->Destroy();

	// destroy the IO settings object.
	ios->Destroy();

	// Destroy the SDK manager and all the other objects it was handling.
	fbxSdk->Destroy();
	return ret;
}


void dfbxImport::PopulateScene (FbxScene* const fbxScene, dPluginScene* const ngdScene)
{
	NodeMap nodeMap;
	MeshMap meshCache;
	MaterialGlobalMap materialCache;

	dScene::dTreeNode* const defaulMaterialNode = ngdScene->CreateMaterialNode(DEFUALT_MATERIAL_ID);
	dMaterialNodeInfo* const defaulMaterial = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(defaulMaterialNode);
	defaulMaterial->SetName("default_material");

	m_materialId = 0;
	LoadHierarchy  (fbxScene, ngdScene, nodeMap);

	NodeMap::Iterator iter (nodeMap);
	for (iter.Begin(); iter; iter ++) {
		FbxNode* const fbxNode = iter.GetKey(); 
		dScene::dTreeNode* const node = iter.GetNode()->GetInfo(); 
		FbxNodeAttribute* const attribute = fbxNode->GetNodeAttribute();
		if (attribute) {
			FbxNodeAttribute::EType attributeType = attribute->GetAttributeType();
			switch (attributeType)
			{
				case FbxNodeAttribute::eMesh: 
				{
					ImportMeshNode (fbxScene, ngdScene, fbxNode, node, meshCache, materialCache);
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
		}
	}
}


void dfbxImport::LoadHierarchy  (FbxScene* const fbxScene, dPluginScene* const ngdScene, NodeMap& nodeMap)
{
	dList <ImportStackData> nodeStack; 

	// Print the nodes of the scene and their attributes recursively.
	FbxNode* const rootNode = fbxScene->GetRootNode();
	if(rootNode) {
		int count = rootNode->GetChildCount();
		for(int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData (GetIdentityMatrix(), rootNode->GetChild(count - i - 1), ngdScene->GetRootNode()));
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

		nodeMap.Insert (node, data.m_fbxNode);

		int count = data.m_fbxNode->GetChildCount();
		for(int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData (matrix, data.m_fbxNode->GetChild(count - i - 1), node));	
		}
	}
}


void dfbxImport::ImportMeshNode (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const node, MeshMap& meshCache, MaterialGlobalMap& materialCache)
{
	dTree<dPluginScene::dTreeNode*, FbxMesh*>::dTreeNode* instanceNode = meshCache.Find(fbxMeshNode->GetMesh());
	if (instanceNode) {
		dScene::dTreeNode* const meshInstance = instanceNode->GetInfo();
		ngdScene->AddReference (node, meshInstance);
	} else {
		FbxMesh* const fbxMesh = fbxMeshNode->GetMesh();
		dScene::dTreeNode* const meshNode = ngdScene->CreateMeshNode(node);
		meshCache.Insert (meshNode, fbxMesh);

		dMeshNodeInfo* const instance = (dMeshNodeInfo*) ngdScene->GetInfoFromNode(meshNode);
		char name[256];
		sprintf (name, "%s_mesh", fbxMeshNode->GetName());
		instance->SetName(name);

		FbxAMatrix pivotMatrix;
		fbxMesh->GetPivot(pivotMatrix);
		instance->SetPivotMatrix(dMatrix (pivotMatrix));

		MaterialLocalMap localMaterialIndex;
		ImportMaterials (fbxScene, ngdScene, fbxMeshNode, meshNode, materialCache, localMaterialIndex);

//		int materialID = 0;		
//		dScene::dTreeNode* const matNode = ngdScene->CreateMaterialNode(materialID);
//		dMaterialNodeInfo* const material = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(matNode);
//		material->SetName("default material");
//		materialID ++;

		int faceCount = fbxMesh->GetPolygonCount();
		int indexCount = 0;
		for (int i = 0; i < faceCount; i ++) {
			indexCount += fbxMesh->GetPolygonSize(i);
		}

		int* const faceIndexList = new int [faceCount];
		int* const materialIndex = new int [faceCount];
		int* const vertexIndex = new int [indexCount];
		int* const normalIndex = new int [indexCount];
		int* const uv0Index = new int [indexCount];
		int* const uv1Index = new int [indexCount];

		dVector* const vertexArray = new dVector[fbxMesh->GetControlPointsCount()]; 
		dVector* const normalArray = new dVector[indexCount];
		dVector* const uv0Array = new dVector[indexCount];
		dVector* const uv1Array = new dVector[indexCount];
		
		const FbxVector4* const controlPoints = fbxMesh->GetControlPoints();
		for (int i = 0; i < fbxMesh->GetControlPointsCount(); i ++) {
			const FbxVector4& p = controlPoints[i];
			vertexArray[i] = dVector (dFloat(p[0]), dFloat(p[1]), dFloat(p[2]), 0.0f);
		}

		FbxGeometryElementUV* const uvArray = fbxMesh->GetElementUV ();
		FbxLayerElement::EMappingMode uvMapingMode = uvArray ? uvArray->GetMappingMode() : FbxGeometryElement::eNone;
		FbxLayerElement::EReferenceMode uvRefMode = uvArray ? uvArray->GetReferenceMode() : FbxGeometryElement::eIndex;

		FbxGeometryElementMaterial* const materialArray = fbxMesh->GetElementMaterial();
		FbxLayerElement::EMappingMode materialMapingMode = materialArray ? materialArray->GetMappingMode() : FbxGeometryElement::eNone;
		FbxLayerElement::EReferenceMode materialRefMode = materialArray ? materialArray->GetReferenceMode() : FbxGeometryElement::eIndex;

		int index = 0;
		for (int i = 0; i < faceCount; i ++) {
			int polygonIndexCount = fbxMesh->GetPolygonSize(i);

			int materialID = DEFUALT_MATERIAL_ID;
			if (materialArray) {
				if (materialMapingMode == FbxGeometryElement::eByPolygon) {
					materialID = (materialRefMode == FbxGeometryElement::eDirect) ? i : materialArray->GetIndexArray().GetAt(i);
				} else {
					materialID = (materialRefMode == FbxGeometryElement::eDirect) ? 0 : materialArray->GetIndexArray().GetAt(0);
				}
			}
			MaterialLocalMap::dTreeNode* const matNode =  localMaterialIndex.Find(materialID);
			dAssert (matNode);
			dMaterialNodeInfo* const material = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(matNode->GetInfo());
			materialIndex[i] = material->GetId();

			faceIndexList[i] = polygonIndexCount;
			for (int j = 0; j < polygonIndexCount; j ++) {
				vertexIndex[index] = fbxMesh->GetPolygonVertex (i, j);
				FbxVector4 n(0, 1, 0, 0);
				fbxMesh->GetPolygonVertexNormal (i, j, n);
				normalArray[index] = dVector (dFloat(n[0]), dFloat(n[1]), dFloat(n[2]), 0.0f);
				normalIndex[index] = index;

				FbxVector2 uv(0, 0);
				if (uvMapingMode == FbxGeometryElement::eByPolygonVertex) {
					int textIndex = (uvRefMode == FbxGeometryElement::eDirect) ? index :  uvArray->GetIndexArray().GetAt(index);
					uv = uvArray->GetDirectArray().GetAt(textIndex);
				}
				uv0Index[index] = index;
				uv0Array[index] = dVector (dFloat(uv[0]), dFloat(uv[1]), 0.0f, 0.0f);
				
				uv1Index[index] = 0;
				uv1Array[index] = dVector (0.0f, 0.0f, 0.0f, 0.0f);

				index ++;
				dAssert (index <= indexCount);
			}
		}

		instance->BuildFromVertexListIndexList(faceCount, faceIndexList, materialIndex, 
											   &vertexArray[0].m_x, sizeof (dVector), vertexIndex, 
											   &normalArray[0].m_x, sizeof (dVector), normalIndex,
											   &uv0Array[0].m_x, sizeof (dVector), uv0Index,
											   &uv1Array[0].m_x, sizeof (dVector), uv1Index);

		delete[] uv1Array;
		delete[] uv0Array;
		delete[] normalArray;
		delete[] vertexArray;
		delete[] uv1Index;
		delete[] uv0Index;
		delete[] normalIndex;
		delete[] vertexIndex;
		delete[] materialIndex;
		delete[] faceIndexList;
	}
}


void dfbxImport::ImportMaterials (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const meshNode, MaterialGlobalMap& materialCache, MaterialLocalMap& localMaterilIndex)
{
	int materialCount = fbxMeshNode->GetMaterialCount();
	if (materialCount) { 
		for (int i = 0; i < materialCount; i ++) {
			FbxSurfaceMaterial* const fbxMaterial = fbxMeshNode->GetMaterial(i);
			MaterialGlobalMap::dTreeNode* matNode = materialCache.Find(fbxMaterial);
			if (!matNode) {
				dScene::dTreeNode* const node = ngdScene->CreateMaterialNode(m_materialId);
				matNode = materialCache.Insert (node, fbxMaterial);

				dMaterialNodeInfo* const material = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(node);
				material->SetName(fbxMaterial->GetName());

				m_materialId ++;
			}
			dScene::dTreeNode* const materialNode = matNode->GetInfo();
			localMaterilIndex.Insert(materialNode, i);
			ngdScene->AddReference(meshNode, materialNode);
		}
	} else {
		dPluginScene::dTreeNode* const materialNode = ngdScene->FindMaterialById(DEFUALT_MATERIAL_ID);
		localMaterilIndex.Insert(materialNode, DEFUALT_MATERIAL_ID);
		ngdScene->AddReference(meshNode, materialNode);
	}


	
/*
	dMaterialNodeInfo* const material = new dMaterialNodeInfo ();
	char name[256];
	if (mtl->NumSubMtls()) {
		sprintf (name, "%s", maxMaterial->GetName());
	} else {
		sprintf (name, "%s", mtl->GetName());
	}
	material->SetName(name);

	Color ambient (maxMaterial->GetAmbient());
	Color difusse (maxMaterial->GetDiffuse());
	Color specular (maxMaterial->GetSpecular());
	float shininess (maxMaterial->GetShininess());
	float shininessStr (maxMaterial->GetShinStr());      
	float tranparency (maxMaterial->GetXParency());

	material->SetAmbientColor (dVector (ambient.r, ambient.g, ambient.b, 1.0f));
	material->SetDiffuseColor (dVector (difusse.r, difusse.g, difusse.b, 1.0f));
	material->SetSpecularColor(dVector (specular.r * shininess, specular.g * shininess, specular.b * shininess, 1.0f));
	material->SetShininess(shininessStr * 100.0f);
	material->SetOpacity (1.0f - tranparency);

	dScene::dTreeNode* textNode = NULL;
	Texmap* const tex = maxMaterial->GetSubTexmap(1);
	if (tex) {
		Class_ID texId (tex->ClassID());
		if (texId != Class_ID(BMTEX_CLASS_ID, 0x00)) {
			_ASSERTE (0);
			//					continue;
		}
		BitmapTex* const bitmapTex = (BitmapTex *)tex;
		const char* const texPathName = bitmapTex->GetMapName();

		const char* texName = strrchr (texPathName, '\\');
		if (texName) {
			texName ++;
		} else {
			texName = strrchr (texPathName, '/');
			if (texName) {
				texName ++;
			} else {
				texName = texPathName;
			}
		}

		dCRCTYPE crc = dCRC64(texName);
		dTree<dScene::dTreeNode*, dCRCTYPE>::dTreeNode* cacheNode = imageCache.Find(crc);
		if (!cacheNode) {
			dScene::dTreeNode* const textNode = scene.CreateTextureNode(texName);
			cacheNode = imageCache.Insert(textNode, crc);
		}
		textNode = cacheNode->GetInfo();
		//scene.AddReference(matNode, textNode);
		dTextureNodeInfo* const texture = (dTextureNodeInfo*) scene.GetInfoFromNode(textNode);
		texture->SetName(texName);
		material->SetDiffuseTextId(texture->GetId());
		material->SetAmbientTextId(texture->GetId());
		material->SetSpecularTextId(texture->GetId());
	}
	dCRCTYPE signature = material->CalculateSignature();
	dScene::dTreeNode* matNode = scene.FindMaterialBySignature(signature);
	if (!matNode) {
		matNode = scene.AddNode(material, scene.GetMaterialCacheNode());
		material->SetId(materialID);
		materialID ++;
		if (textNode) {
			scene.AddReference(matNode, textNode);
		}
	}
	scene.AddReference(meshNode, matNode);
	material->Release();

	dMaterialNodeInfo* const cachedMaterialInfo = (dMaterialNodeInfo*)scene.GetInfoFromNode(matNode);
	int id = cachedMaterialInfo->GetId();
	for (int j = 0; j < facesCount; j ++) {
		MNFace* const maxFace = maxMesh.F(j);
		if (maxFace->material == i) {
			materialIndex[j] = id;
		}
	}
*/
}
