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

dPluginInterface* dfbxImport::m_interface;

dImportPlugin* dfbxImport::GetPluginFBX()
{
	static dfbxImport plugin("*.fbx", "Import Autodesk fbx file (.fbx)", "fbx mesh import");
	return &plugin;
}

dImportPlugin* dfbxImport::GetPluginOBJ()
{
	static dfbxImport plugin("*.obj", "Import Wavefront obj file (.obj)", "obj mesh import");
	return &plugin;
}

dImportPlugin* dfbxImport::GetPluginDAE()
{
	static dfbxImport plugin("*.dae", "Import Autodesk Collada file (.dae)", "dae mesh import");
	return &plugin;
}


dfbxImport::dfbxImport (const char* const ext, const char* const signature, const char* const description)
	:dImportPlugin() 
	, m_materialId (0)
{
	strcpy (m_ext, ext);
	strcpy (m_signature, signature);
	strcpy (m_description, description);
}


bool dfbxImport::Import (const char* const fileName, dPluginInterface* const interface)
{
	bool ret = false;

	m_interface = interface;

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

	// set the progress callback
	fbxImporter->SetProgressCallback (ProgressCallback);

	// Use the first argument as the filename for the importer.
	if(fbxImporter->Initialize(fileName, -1, fbxSdk->GetIOSettings())) { 
		ret = true;
		// Create a new scene so that it can be populated by the imported file.
		FbxScene* const fbxScene = FbxScene::Create(fbxSdk,"myScene");

		// Import the contents of the file into the scene.
		fbxImporter->Import(fbxScene);

		// Convert the scene to meters using the defined options.
		FbxGlobalSettings& settings = fbxScene->GetGlobalSettings();

		dMatrix convertMatrix (dGetIdentityMatrix());

		const FbxSystemUnit& systemUnit = settings.GetSystemUnit();
		dFloat scaleFactor = systemUnit.GetScaleFactor();
		convertMatrix[0][0] = dFloat (scaleFactor / 100.0f);
		convertMatrix[1][1] = dFloat (scaleFactor / 100.0f);
		convertMatrix[2][2] = dFloat (scaleFactor / 100.0f);

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

		dMatrix axisMatrix (dGetIdentityMatrix());
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

bool dfbxImport::ProgressCallback (float pPercentage, FbxString pStatus)
{
	return  m_interface->UpdateProgress(pPercentage * 0.01f);
}



void dfbxImport::PopulateScene (FbxScene* const fbxScene, dPluginScene* const ngdScene)
{
	GlobalNoceMap nodeMap;
	GlobalMeshMap meshCache;
	GlobalTextureMap textureCache;
	GlobalMaterialMap materialCache;
	UsedMaterials usedMaterials;

	dScene::dTreeNode* const defaulMaterialNode = ngdScene->CreateMaterialNode(DEFUALT_MATERIAL_ID);
	dMaterialNodeInfo* const defaulMaterial = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(defaulMaterialNode);
	defaulMaterial->SetName("default_material");
	usedMaterials.Insert(0, defaulMaterialNode);

	m_materialId = 0;
	LoadHierarchy  (fbxScene, ngdScene, nodeMap);

	GlobalNoceMap::Iterator iter (nodeMap);
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
					ImportMeshNode (fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials, nodeMap);
					break;
				}

				case FbxNodeAttribute::eSkeleton: 
				{
					ImportSkeleton (fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
					break;
				}

				case FbxNodeAttribute::eLine:
				{
					ImportLineShape (fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
					break;
				}

				case FbxNodeAttribute::eNurbsCurve:
				{
					ImportNurbCurveShape (fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
					break;
				}

				case FbxNodeAttribute::eNull:
				{
					break;
				}


				
				case FbxNodeAttribute::eMarker:
				case FbxNodeAttribute::eNurbs: 
				case FbxNodeAttribute::ePatch:
				case FbxNodeAttribute::eCamera: 
				case FbxNodeAttribute::eCameraStereo:
				case FbxNodeAttribute::eCameraSwitcher:
				case FbxNodeAttribute::eLight:
				case FbxNodeAttribute::eOpticalReference:
				case FbxNodeAttribute::eOpticalMarker:
				
				case FbxNodeAttribute::eTrimNurbsSurface:
				case FbxNodeAttribute::eBoundary:
				case FbxNodeAttribute::eNurbsSurface:
				case FbxNodeAttribute::eShape:
				case FbxNodeAttribute::eLODGroup:
				case FbxNodeAttribute::eSubDiv:
				case FbxNodeAttribute::eCachedEffect:
				case FbxNodeAttribute::eUnknown:
				default:
					dAssert(0);
					break;
			}
		}
	}

	
	UsedMaterials::Iterator iter1 (usedMaterials);
	for (iter1.Begin(); iter1; iter1 ++) {
		int count = iter1.GetNode()->GetInfo();
		if (!count) {
			dScene::dTreeNode* const materiaCacheNode = ngdScene->FindGetMaterialCacheNode();
			dScene::dTreeNode* const materialNode = iter1.GetKey();
			void* nextLink;
			for (void* link = ngdScene->GetFirstParentLink(materialNode); link; link = nextLink) {
				nextLink = ngdScene->GetNextParentLink(materialNode, link);
				dScene::dTreeNode* const parentNode = ngdScene->GetNodeFromLink(link);
				if (parentNode != materiaCacheNode) {
					ngdScene->RemoveReference (parentNode, materialNode);
				}
			}
		}
	}
}




void dfbxImport::ImportMaterials (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const meshNode, 
								  GlobalMaterialMap& materialCache, LocalMaterialMap& localMaterilIndex, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials)
{
	dPluginScene::dTreeNode* const materialNode = ngdScene->FindMaterialById(DEFUALT_MATERIAL_ID);
	localMaterilIndex.Insert(materialNode, DEFUALT_MATERIAL_ID);
	ngdScene->AddReference(meshNode, materialNode);

	int materialCount = fbxMeshNode->GetMaterialCount();
	if (materialCount) { 
		for (int i = 0; i < materialCount; i ++) {
			FbxSurfaceMaterial* const fbxMaterial = fbxMeshNode->GetMaterial(i);
			GlobalMaterialMap::dTreeNode* globalMaterialCacheNode = materialCache.Find(fbxMaterial);
			if (!globalMaterialCacheNode) {
				dScene::dTreeNode* const materialNode = ngdScene->CreateMaterialNode(m_materialId);
				globalMaterialCacheNode = materialCache.Insert (materialNode, fbxMaterial);
				usedMaterials.Insert(0, materialNode);

				dMaterialNodeInfo* const materialInfo = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(materialNode);
				materialInfo->SetName(fbxMaterial->GetName());

				dVector ambient(materialInfo->GetAmbientColor());
				dVector difusse(materialInfo->GetDiffuseColor());
				dVector specular(materialInfo->GetSpecularColor());
				dFloat opacity = materialInfo->GetOpacity();
				dFloat shininess = materialInfo->GetShininess();
				if (fbxMaterial->GetClassId().Is(FbxSurfacePhong::ClassId))	{
					FbxSurfacePhong* const phongMaterial = (FbxSurfacePhong *) fbxMaterial;
				
					// Get the ambient Color
					ambient[0] = dFloat(phongMaterial->Ambient.Get()[0]);
					ambient[1] = dFloat(phongMaterial->Ambient.Get()[1]);
					ambient[2] = dFloat(phongMaterial->Ambient.Get()[2]);
					ambient[3] = 1.0f;
			
					// get the Diffuse Color
					difusse[0] = dFloat(phongMaterial->Diffuse.Get()[0]);
					difusse[1] = dFloat(phongMaterial->Diffuse.Get()[1]);
					difusse[2] = dFloat(phongMaterial->Diffuse.Get()[2]);
					difusse[3] = 1.0f;

					// Get specular Color (unique to Phong materials)
					specular[0] = dFloat(phongMaterial->Specular.Get()[0]);
					specular[1] = dFloat(phongMaterial->Specular.Get()[1]);
					specular[2] = dFloat(phongMaterial->Specular.Get()[2]);
					specular[3] = 1.0f;

					// Display the Shininess
					shininess = dFloat(phongMaterial->Shininess.Get() * 100.0f);

					// Display the Emissive Color
					//lKFbxDouble3 =((FbxSurfacePhong *) lMaterial)->Emissive;
					//theColor.Set(lKFbxDouble3.Get()[0], lKFbxDouble3.Get()[1], lKFbxDouble3.Get()[2]);
					//DisplayColor("            Emissive: ", theColor);

					//Opacity is Transparency factor now
					opacity = dFloat(1.0 - phongMaterial->TransparencyFactor.Get());

					// Display the Reflectivity
					//lKFbxDouble1 =((FbxSurfacePhong *) lMaterial)->ReflectionFactor;
					//DisplayDouble("            Reflectivity: ", lKFbxDouble1.Get());
				} else {
					dAssert (0);
				}

				materialInfo->SetAmbientColor (ambient);
				materialInfo->SetDiffuseColor (difusse);
				materialInfo->SetSpecularColor(specular);
				materialInfo->SetShininess(shininess);
				materialInfo->SetOpacity (opacity);

				m_materialId ++;
			}
			dScene::dTreeNode* const materialNode = globalMaterialCacheNode->GetInfo();
			localMaterilIndex.Insert(materialNode, i);
			ngdScene->AddReference(meshNode, materialNode);

			// assume layer 0 for now
			FbxProperty textureProperty (fbxMaterial->FindProperty(FbxLayerElement::sTextureChannelNames[0]));
			if (textureProperty.IsValid()) {
				ImportTexture (ngdScene, textureProperty, materialNode, textureCache);
			}
		}
	}
}


void dfbxImport::ImportTexture (dPluginScene* const ngdScene, FbxProperty pProperty, dPluginScene::dTreeNode* const materialNode, GlobalTextureMap& textureCache)
{
	int lTextureCount = pProperty.GetSrcObjectCount<FbxTexture>();
	for (int j = 0; j < lTextureCount; ++j) {
		//Here we have to check if it's layered textures, or just textures:
		FbxLayeredTexture *lLayeredTexture = pProperty.GetSrcObject<FbxLayeredTexture>(j);
		if (lLayeredTexture)
		{
			dAssert (0);
/*
			DisplayInt("    Layered Texture: ", j);
			FbxLayeredTexture *lLayeredTexture = pProperty.GetSrcObject<FbxLayeredTexture>(j);
			int lNbTextures = lLayeredTexture->GetSrcObjectCount<FbxTexture>();
			for(int k =0; k<lNbTextures; ++k)
			{
				FbxTexture* lTexture = lLayeredTexture->GetSrcObject<FbxTexture>(k);
				if(lTexture)
				{

					if(pDisplayHeader){                    
						DisplayInt("    Textures connected to Material ", pMaterialIndex);
						pDisplayHeader = false;
					}

					//NOTE the blend mode is ALWAYS on the LayeredTexture and NOT the one on the texture.
					//Why is that?  because one texture can be shared on different layered textures and might
					//have different blend modes.

					FbxLayeredTexture::EBlendMode lBlendMode;
					lLayeredTexture->GetTextureBlendMode(k, lBlendMode);
					DisplayString("    Textures for ", pProperty.GetName());
					DisplayInt("        Texture ", k);  
					DisplayTextureInfo(lTexture, (int) lBlendMode);   
				}
			}
*/
		} else {
			//no layered texture simply get on the property
			FbxTexture* const texture = pProperty.GetSrcObject<FbxTexture>(j);
			if(texture) {
				GlobalTextureMap::dTreeNode* textCacheNode = textureCache.Find(texture);
				if (!textCacheNode) {

					//const FbxString& name = pProperty.GetName();
					//const char* const texPathName = name.Buffer();
					dAssert (pProperty.GetName() == "DiffuseColor");

					FbxFileTexture* const fileTexture = FbxCast<FbxFileTexture>(texture);
					//FbxProceduralTexture* const proceduralTexture = FbxCast<FbxProceduralTexture>(texture);
					dAssert (!FbxCast<FbxProceduralTexture>(texture));

					const char* const texPathName = fileTexture->GetFileName();

/*
					DisplayDouble("            Scale U: ", pTexture->GetScaleU());
					DisplayDouble("            Scale V: ", pTexture->GetScaleV());
					DisplayDouble("            Translation U: ", pTexture->GetTranslationU());
					DisplayDouble("            Translation V: ", pTexture->GetTranslationV());
					DisplayBool("            Swap UV: ", pTexture->GetSwapUV());
					DisplayDouble("            Rotation U: ", pTexture->GetRotationU());
					DisplayDouble("            Rotation V: ", pTexture->GetRotationV());
					DisplayDouble("            Rotation W: ", pTexture->GetRotationW());
					const char* lAlphaSources[] = { "None", "RGB Intensity", "Black" };
					DisplayString("            Alpha Source: ", lAlphaSources[pTexture->GetAlphaSource()]);
					DisplayDouble("            Cropping Left: ", pTexture->GetCroppingLeft());
					DisplayDouble("            Cropping Top: ", pTexture->GetCroppingTop());
					DisplayDouble("            Cropping Right: ", pTexture->GetCroppingRight());
					DisplayDouble("            Cropping Bottom: ", pTexture->GetCroppingBottom());
					const char* lMappingTypes[] = { "Null", "Planar", "Spherical", "Cylindrical", "Box", "Face", "UV", "Environment" };
					DisplayString("            Mapping Type: ", lMappingTypes[pTexture->GetMappingType()]);
					if (pTexture->GetMappingType() == FbxTexture::ePlanar) {
						const char* lPlanarMappingNormals[] = { "X", "Y", "Z" };

						DisplayString("            Planar Mapping Normal: ", lPlanarMappingNormals[pTexture->GetPlanarMappingNormal()]);
					}

					const char* lBlendModes[]   = { "Translucent", "Add", "Modulate", "Modulate2" };   
					if(pBlendMode >= 0)
						DisplayString("            Blend Mode: ", lBlendModes[pBlendMode]);
					DisplayDouble("            Alpha: ", pTexture->GetDefaultAlpha());

					if (lFileTexture) {
						const char* lMaterialUses[] = { "Model Material", "Default Material" };
						DisplayString("            Material Use: ", lMaterialUses[lFileTexture->GetMaterialUse()]);
					}

					const char* pTextureUses[] = { "Standard", "Shadow Map", "Light Map", "Spherical Reflexion Map", "Sphere Reflexion Map", "Bump Normal Map" };
					DisplayString("            Texture Use: ", pTextureUses[pTexture->GetTextureUse()]);
					DisplayString("");                

*/
					char path[2048];
					const char* const texName = dGetNameFromPath(texPathName);
					dExtractPathFromFullName (texPathName, path);

					dScene::dTreeNode* const textNode = ngdScene->CreateTextureNode(texName);
					dTextureNodeInfo* const textureInfo = (dTextureNodeInfo*) ngdScene->GetInfoFromNode(textNode);
					textureInfo->SetName(texName);
					textureInfo->SetPathName(texName);
					textCacheNode = textureCache.Insert (textNode, texture);
				}

				// for now only set the Diffuse texture
				dScene::dTreeNode* const textNode = textCacheNode->GetInfo();
				dTextureNodeInfo* const textureInfo = (dTextureNodeInfo*) ngdScene->GetInfoFromNode(textNode);
				dMaterialNodeInfo* const materialInfo = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(materialNode);
				ngdScene->AddReference (materialNode, textNode);

				materialInfo->SetDiffuseTextId(textureInfo->GetId());
				materialInfo->SetAmbientTextId(textureInfo->GetId());
				materialInfo->SetSpecularTextId(textureInfo->GetId());
			}
		}
	}
}


void dfbxImport::LoadHierarchy  (FbxScene* const fbxScene, dPluginScene* const ngdScene, GlobalNoceMap& nodeMap)
{
	dList <ImportStackData> nodeStack; 

	FbxAnimEvaluator* const evaluator = fbxScene->GetEvaluator();

	// Print the nodes of the scene and their attributes recursively.
	FbxNode* const rootNode = fbxScene->GetRootNode();
	if(rootNode) {
		int count = rootNode->GetChildCount();
		for(int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData (dGetIdentityMatrix(), rootNode->GetChild(count - i - 1), ngdScene->GetRootNode()));
		}
	}

	while (nodeStack.GetCount()) {

		ImportStackData data (nodeStack.GetLast()->GetInfo());
		nodeStack.Remove(nodeStack.GetLast());

		dPluginScene::dTreeNode* const node = ngdScene->CreateSceneNode (data.m_parentNode);
		dSceneNodeInfo* const info = (dSceneNodeInfo*) ngdScene->GetInfoFromNode (node);

		dMatrix localMatrix (evaluator->GetNodeLocalTransform(data.m_fbxNode));

		info->SetName(data.m_fbxNode->GetName());
		info->SetTransform (localMatrix);

		FbxVector4 fbxPivotScaling (data.m_fbxNode->GetGeometricScaling(FbxNode::eSourcePivot));
		FbxVector4 fbxPivotRotation (data.m_fbxNode->GetGeometricRotation(FbxNode::eSourcePivot));
		FbxVector4 fbxPivotTranslation (data.m_fbxNode->GetGeometricTranslation(FbxNode::eSourcePivot));

		dVector pivotTranslation (dFloat(fbxPivotTranslation[0]), dFloat(fbxPivotTranslation[1]), dFloat(fbxPivotTranslation[2]), 1.0f);
		dVector pivotRotation (dFloat(fbxPivotRotation[0] * 3.14159265359 / 180.0), dFloat(fbxPivotRotation[1] * 3.14159265359 / 180.0), dFloat(fbxPivotRotation[2] * 3.14159265359 / 180.0), 0.0f);

		dMatrix pivotScale (dGetIdentityMatrix());
		pivotScale[0][0] = dFloat(fbxPivotScaling[0]);
		pivotScale[1][1] = dFloat(fbxPivotScaling[1]);
		pivotScale[2][2] = dFloat(fbxPivotScaling[2]);
		dMatrix pivotMatrix (pivotScale * dMatrix(pivotRotation[0], pivotRotation[1], pivotRotation[2], pivotTranslation));
		info->SetGeometryTransform(pivotMatrix);

		nodeMap.Insert (node, data.m_fbxNode);

		int count = data.m_fbxNode->GetChildCount();
		for(int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData (dGetIdentityMatrix(), data.m_fbxNode->GetChild(count - i - 1), node));	
		}
	}
}


void dfbxImport::ImportLineShape (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials)
{
	GlobalMeshMap::dTreeNode* instanceNode = meshCache.Find(fbxNode->GetLine());
	if (instanceNode) {
dAssert (0);
		dScene::dTreeNode* const meshInstance = instanceNode->GetInfo();
		ngdScene->AddReference (node, meshInstance);
	} else {
		FbxLine* const fbxLine = fbxNode->GetLine();

		dScene::dTreeNode* const lineNode = ngdScene->CreateLineNode(node);
		meshCache.Insert (lineNode, fbxLine);

		dLineNodeInfo* const instance = (dLineNodeInfo*) ngdScene->GetInfoFromNode (lineNode);

		char name[256];
		sprintf (name, "%s_line", fbxNode->GetName());
		instance->SetName(name);

		FbxAMatrix pivotMatrix;
		fbxLine->GetPivot(pivotMatrix);

		dMatrix matrix (pivotMatrix);
		instance->SetPivotMatrix(matrix);
		dAssert (matrix[1][1] == 1.0f);

	    //int pointsCount = fbxLine->GetControlPointsCount();
		const FbxVector4* const controlPoints = fbxLine->GetControlPoints();
dAssert (0);		
		const int indexCount = fbxLine->GetIndexArraySize();
		dVector* const vertexArray = new dVector[indexCount]; 
		for (int i = 0; i < indexCount; i ++) {
			int index = fbxLine->GetPointIndexAt(i);
			const FbxVector4& p = controlPoints[index];
			vertexArray[i] = dVector (dFloat(p[0]), dFloat(p[1]), dFloat(p[2]), 0.0f);
		}
		
/*
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
			LocalMaterialMap::dTreeNode* const matNode = localMaterialIndex.Find(materialID);
			dAssert (matNode);
			dMaterialNodeInfo* const material = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(matNode->GetInfo());
			materialIndex[i] = material->GetId();

			dAssert (usedMaterials.Find(matNode->GetInfo()));
			usedMaterials.Find(matNode->GetInfo())->GetInfo() += 1;

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

		// some meshes has degenerated faces we must repair them to be legal manifold
		instance->RepairTJoints();

		delete[] uv1Array;
		delete[] uv0Array;
		delete[] normalArray;
		
		delete[] uv1Index;
		delete[] uv0Index;
		delete[] normalIndex;
		delete[] vertexIndex;
		delete[] materialIndex;
		delete[] faceIndexList;
*/
		delete[] vertexArray;
	}
}


void dfbxImport::ImportNurbCurveShape (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials)
{
	GlobalMeshMap::dTreeNode* instanceNode = meshCache.Find(fbxNode->GetLine());
	if (instanceNode) {
		dScene::dTreeNode* const meshInstance = instanceNode->GetInfo();
		ngdScene->AddReference (node, meshInstance);
	} else {
		FbxNurbsCurve* const fbxCurve = fbxNode->GetNurbsCurve();

		dScene::dTreeNode* const lineNode = ngdScene->CreateLineNode(node);
		meshCache.Insert (lineNode, fbxCurve);

		dLineNodeInfo* const instance = (dLineNodeInfo*) ngdScene->GetInfoFromNode (lineNode);

		char name[256];
		sprintf (name, "%s_line", fbxNode->GetName());
		instance->SetName(name);

		FbxAMatrix pivotMatrix;
		fbxCurve->GetPivot(pivotMatrix);

		dMatrix matrix (pivotMatrix);
		instance->SetPivotMatrix(matrix);
		dAssert (matrix[1][1] == 1.0f);

		dAssert ((fbxCurve->GetOrder() - 1) <= 3); 
		int degree = fbxCurve->GetOrder() - 1;

//		bool xxx = fbxCurve->IsBezier();

	    int pointsCount = fbxCurve->GetControlPointsCount();
		const FbxVector4* const controlPoints = fbxCurve->GetControlPoints();
		dBigVector* const vertexArray = new dBigVector[pointsCount + 1]; 

		for (int i = 0; i < pointsCount; i ++) {
			const FbxVector4& p = controlPoints[i];
			vertexArray[i] = dVector (dFloat(p[0]), dFloat(p[1]), dFloat(p[2]), 1.0f);
		}
		FbxNurbsCurve::EType type = fbxCurve->GetType();
		if ((type == FbxNurbsCurve::eClosed) || (type == FbxNurbsCurve::ePeriodic)) {
			vertexArray[pointsCount] = vertexArray[0];
			pointsCount ++;
		}


		const int knotCount = fbxCurve->GetKnotCount();
		const double* const knots = fbxCurve->GetKnotVector();
		double scale = 1.0 / knots[knotCount - 1];
		dAssert (pointsCount == knotCount - 2 * (degree - 1));

		dFloat64* const knotArray = new dFloat64[pointsCount]; 
		for (int i = 0; i < knotCount - degree * 2; i ++) {
			double val = knots[i + degree];
			knotArray[i] = dFloat (val * scale);
		}

		instance->m_curve.CreateFromKnotVectorAndControlPoints(degree, knotCount - degree * 2, knotArray, vertexArray);

		delete[] knotArray;
		delete[] vertexArray;
	}
}

void dfbxImport::ImportSkeleton (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials)
{
/*
	GlobalMeshMap::dTreeNode* instanceNode = meshCache.Find(fbxNode->GetLine());
	if (instanceNode) {
		dScene::dTreeNode* const meshInstance = instanceNode->GetInfo();
		ngdScene->AddReference (node, meshInstance);
	} else {
		FbxSkeleton* const fbxSkeleton = fbxNode->GetSkeleton();
		dAssert (fbxSkeleton);

		char name[256];
		sprintf (name, "%s", fbxNode->GetName());
		sprintf (name, "%s", fbxNode->GetName());
	}
*/

}


void dfbxImport::ImportMeshNode (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, GlobalNoceMap& nodeMap)
{
	GlobalMeshMap::dTreeNode* instanceNode = meshCache.Find(fbxMeshNode->GetMesh());
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

		dMatrix matrix (pivotMatrix);
		instance->SetPivotMatrix(matrix);
		dAssert (matrix[1][1] == 1.0f);

		LocalMaterialMap localMaterialIndex;
		ImportMaterials (fbxScene, ngdScene, fbxMeshNode, meshNode, materialCache, localMaterialIndex, textureCache, usedMaterials);

		//int materialID = 0;		
		//dScene::dTreeNode* const matNode = ngdScene->CreateMaterialNode(materialID);
		//dMaterialNodeInfo* const material = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(matNode);
		//material->SetName("default material");
		//materialID ++;

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
			LocalMaterialMap::dTreeNode* const matNode = localMaterialIndex.Find(materialID);
			dAssert (matNode);
			dMaterialNodeInfo* const material = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(matNode->GetInfo());
			materialIndex[i] = material->GetId();

			dAssert (usedMaterials.Find(matNode->GetInfo()));
			usedMaterials.Find(matNode->GetInfo())->GetInfo() += 1;

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

		// some meshes has degenerated faces we must repair them to be legal manifold
		instance->RepairTJoints();

		// import skin if there is any
		int deformerCount = fbxMesh->GetDeformerCount(FbxDeformer::eSkin);
		for(int i = 0; i < deformerCount; i ++) {

			FbxSkin* const skin = (FbxSkin*)fbxMesh->GetDeformer(i, FbxDeformer::eSkin);
			dAssert (skin);

			// count the number of weights
			int skinVertexDataCount = 0;
			int clusterCount = skin->GetClusterCount();
			for (int i = 0; i < clusterCount; i++) {
				FbxCluster* cluster = skin->GetCluster(i);
				skinVertexDataCount += cluster->GetControlPointIndicesCount();
			}
			dGeometryNodeSkinModifierInfo::dBoneVertexWeightData* const skinVertexData = new dGeometryNodeSkinModifierInfo::dBoneVertexWeightData[skinVertexDataCount];
			
			int actualSkinVertexCount = 0;
			for (int i = 0; i < clusterCount; i++)
			{
				FbxCluster* const cluster = skin->GetCluster(i);
				FbxNode* const fbxBone = cluster->GetLink(); // Get a reference to the bone's node

				//char xxx[256];
				//sprintf (xxx, "%s", fbxBone->GetName());
				GlobalNoceMap::dTreeNode* const boneNode = nodeMap.Find(fbxBone);
				if (boneNode) {
					dPluginScene::dTreeNode* const bone = boneNode->GetInfo();

					// Get the bind pose
					//FbxAMatrix bindPoseMatrix;
					//cluster->GetTransformLinkMatrix(bindPoseMatrix);

					int *boneVertexIndices = cluster->GetControlPointIndices();
					double *boneVertexWeights = cluster->GetControlPointWeights();
					// Iterate through all the vertices, which are affected by the bone
					int numBoneVertexIndices = cluster->GetControlPointIndicesCount();
					for (int j = 0; j < numBoneVertexIndices; j++) 	{
						int boneVertexIndex = boneVertexIndices[j];
						float boneWeight = (float)boneVertexWeights[j];
						skinVertexData[actualSkinVertexCount].m_vertexIndex = boneVertexIndex;
						skinVertexData[actualSkinVertexCount].m_weight = boneWeight;
						skinVertexData[actualSkinVertexCount].m_boneNode = bone;
						actualSkinVertexCount ++;
						dAssert (actualSkinVertexCount <= skinVertexDataCount);
					}
				}
			}

			char skinName[256];
			sprintf (skinName, "%s_skin", fbxMeshNode->GetName());
			dPluginScene::dTreeNode* const skinNode = ngdScene->CreateSkinModifierNode (meshNode);
			dGeometryNodeSkinModifierInfo* const info = (dGeometryNodeSkinModifierInfo*) ngdScene->GetInfoFromNode (skinNode);
			info->SetName(skinName);
			info->SkinMesh (skinNode, ngdScene, skinVertexData, actualSkinVertexCount);

			delete[] skinVertexData;
		}


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

