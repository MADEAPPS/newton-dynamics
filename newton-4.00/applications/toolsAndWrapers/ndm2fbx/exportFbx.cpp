/* Copyright (c) <2003-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "stdafx.h"
#include "exportFbx.h"
#include "exportMesh.h"
#include "exportMeshNode.h"

#ifdef IOS_REF
	#undef  IOS_REF
	#define IOS_REF (*(fbsManager->GetIOSettings()))
#endif

static int InitializeSdkObjects(FbxManager*& fbxManager, FbxScene*& fbxScene);
static void CreateGeometries(const exportMeshNode* const model, FbxScene* const fbxScene);
static FbxMesh* CreateGeometry(const exportMeshNode* const node, FbxScene* const fbxScene);
static FbxNode* CreateSkeleton(const exportMeshNode* const model, FbxScene* const fbxScene);
static void AnimateSkeleton(const exportMeshNode* const model, FbxScene* const fbxScene, FbxNode* const fbxModelRoot);
static bool CreateScene(const exportMeshNode* const model, FbxManager* const fbxManager, FbxScene* const fbxScene);
static bool SaveScene(FbxManager* const fbxManager, FbxDocument* const fbxScene, const char* const filename, int fileFormat = -1, bool embedMedia = false);

bool ExportFbx(const exportMeshNode* const scene, const char* const name)
{
	FbxScene* fbxScene = nullptr;
	FbxManager* fbxManager = nullptr;
	if (!InitializeSdkObjects(fbxManager, fbxScene))
	{
		FBXSDK_printf("failed to initialize fbx sdk: %s\n", name);
		return 0;
	}

	// Create the scene.
	bool lResult = CreateScene(scene, fbxManager, fbxScene);

	if (lResult == false)
	{
		FBXSDK_printf("\n\nAn error occurred while creating the fbsScene...\n");
		fbxManager->Destroy();
		return 0;
	}

	// Save the fbsScene.
	lResult = SaveScene(fbxManager, fbxScene, name);
	if (lResult == false)
	{
		FBXSDK_printf("\n\nAn error occurred while saving the fbsScene...\n");
		fbxManager->Destroy();
		return 0;
	}

	fbxManager->Destroy();
	return true;
}

int InitializeSdkObjects(FbxManager*& fbsManager, FbxScene*& fbsScene)
{
	//The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
	fbsManager = FbxManager::Create();
	if (!fbsManager)
	{
		FBXSDK_printf("Error: Unable to create FBX Manager!\n");
		exit(1);
	}
	else FBXSDK_printf("Autodesk FBX SDK version %s\n", fbsManager->GetVersion());

	//Create an IOSettings object. This object holds all import/export settings.
	FbxIOSettings* ios = FbxIOSettings::Create(fbsManager, IOSROOT);
	fbsManager->SetIOSettings(ios);

	//Load plugins from the executable directory (optional)
	FbxString lPath = FbxGetApplicationDirectory();
	fbsManager->LoadPluginsDirectory(lPath.Buffer());

	//Create an FBX fbsScene. This object holds most objects imported/exported from/to files.
	fbsScene = FbxScene::Create(fbsManager, "My Scene");
	if (!fbsScene)
	{
		FBXSDK_printf("Error: Unable to create FBX fbsScene!\n");
		return 0;
	}
	return 1;
}

bool SaveScene(FbxManager* const fbsManager, FbxDocument* const fbsScene, const char* const name, int fileFormat, bool embedMedia)
{
	int major, minor, revision;
	bool status = true;

	char filename[256];
	sprintf(filename, "%s", name);
	_strlwr(filename);
	char* ptr = strrchr(filename, '.');
	if (ptr && (!strcmp(ptr, ".asf") || !strcmp(ptr, ".amc") || !strcmp(ptr, ".bvh")))
	{
		*ptr = 0;
	}
	strcat(filename, ".fbx");

	// Create an exporter.
	FbxExporter* const fbxExporter = FbxExporter::Create(fbsManager, "");

	if (fileFormat < 0 || fileFormat >= fbsManager->GetIOPluginRegistry()->GetWriterFormatCount())
	{
		// Write in fall back format in less no ASCII format found
		fileFormat = fbsManager->GetIOPluginRegistry()->GetNativeWriterFormat();

		//Try to export in ASCII if possible
		int	lFormatCount = fbsManager->GetIOPluginRegistry()->GetWriterFormatCount();

		for (int lFormatIndex = 0; lFormatIndex < lFormatCount; lFormatIndex++)
		{
			if (fbsManager->GetIOPluginRegistry()->WriterIsFBX(lFormatIndex))
			{
				FbxString lDesc = fbsManager->GetIOPluginRegistry()->GetWriterFormatDescription(lFormatIndex);
				const char *lASCII = "ascii";
				//const char *lASCII = "binary";
				if (lDesc.Find(lASCII) >= 0)
				{
					fileFormat = lFormatIndex;
					break;
				}
			}
		}
	}

	// Set the export states. By default, the export states are always set to 
	// true except for the option eEXPORT_TEXTURE_AS_EMBEDDED. The code below 
	// shows how to change these states.
	IOS_REF.SetBoolProp(EXP_FBX_MATERIAL, true);
	IOS_REF.SetBoolProp(EXP_FBX_TEXTURE, true);
	IOS_REF.SetBoolProp(EXP_FBX_EMBEDDED, embedMedia);
	IOS_REF.SetBoolProp(EXP_FBX_SHAPE, true);
	IOS_REF.SetBoolProp(EXP_FBX_GOBO, true);
	IOS_REF.SetBoolProp(EXP_FBX_ANIMATION, true);
	IOS_REF.SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);

	// Initialize the exporter by providing a filename.
	if (fbxExporter->Initialize(filename, fileFormat, fbsManager->GetIOSettings()) == false)
	{
		FBXSDK_printf("Call to FbxExporter::Initialize() failed.\n");
		FBXSDK_printf("Error returned: %s\n\n", fbxExporter->GetStatus().GetErrorString());
		return false;
	}

	FbxManager::GetFileFormatVersion(major, minor, revision);
	FBXSDK_printf("FBX file format version %d.%d.%d\n\n", major, minor, revision);

	// Export the fbsScene.
	status = fbxExporter->Export(fbsScene);

	// Destroy the exporter.
	fbxExporter->Destroy();
	return status;
}

FbxNode* CreateSkeleton(const exportMeshNode* const model, FbxScene* const fbxScene)
{
	int stack = 1;
	FbxNode* fbxNodesParent[256];
	const exportMeshNode* bvhNodePool[256];
	
	bvhNodePool[0] = model;
	fbxNodesParent[0] = nullptr;

	FbxNode* skeleton = nullptr;
	while (stack)
	{
		stack--;
		const exportMeshNode* const node = bvhNodePool[stack];
		FbxNode* const fbxParent = fbxNodesParent[stack];

		FbxNode* const fbxNode = FbxNode::Create(fbxScene, node->m_name.c_str());
		exportVector posit(node->m_matrix.m_posit);
		exportVector euler1;
		exportVector euler0(node->m_matrix.CalcPitchYawRoll(euler1));
		node->m_fbxNode = fbxNode;

		exportVector euler(euler0.Scale(180.0f / M_PI));
		fbxNode->LclRotation.Set(FbxVector4(euler.m_x, euler.m_y, euler.m_z));
		fbxNode->LclTranslation.Set(FbxVector4(posit.m_x, posit.m_y, posit.m_z));

		FbxSkeleton* const attribute = FbxSkeleton::Create(fbxScene, model->m_name.c_str());
		if (fbxParent)
		{
			attribute->Size.Set(0.1f);
			attribute->SetSkeletonType(FbxSkeleton::eLimbNode);
			fbxParent->AddChild(fbxNode);
		}
		else
		{
			attribute->SetSkeletonType(FbxSkeleton::eRoot);
		}
		fbxNode->SetNodeAttribute(attribute);

		if (!skeleton)
		{
			skeleton = fbxNode;
		}

		for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
			iter != node->m_children.end(); iter++)
		{
			bvhNodePool[stack] = *iter;
			fbxNodesParent[stack] = fbxNode;
			stack++;
		}
	}

	return skeleton;
}


void CreateTexture_____(FbxScene* pScene, FbxMesh* pMesh)
{
	// A texture need to be connected to a property on the material,
	// so let's use the material (if it exists) or create a new one
	FbxSurfacePhong* lMaterial = nullptr;

	//get the node of mesh, add material for it.
	FbxNode* lNode = pMesh->GetNode();
	if (lNode)
	{
		lMaterial = lNode->GetSrcObject<FbxSurfacePhong>(0);
		if (lMaterial == NULL)
		{
			FbxString lMaterialName = "toto";
			FbxString lShadingName = "Phong";
			FbxDouble3 lBlack(0.0, 0.0, 0.0);
			FbxDouble3 lRed(1.0, 0.0, 0.0);
			FbxDouble3 lDiffuseColor(0.75, 0.75, 0.0);
			lMaterial = FbxSurfacePhong::Create(pScene, lMaterialName.Buffer());

			// Generate primary and secondary colors.
			lMaterial->Emissive.Set(lBlack);
			lMaterial->Ambient.Set(lRed);
			lMaterial->AmbientFactor.Set(1.);
			// Add texture for diffuse channel
			lMaterial->Diffuse.Set(lDiffuseColor);
			lMaterial->DiffuseFactor.Set(1.);
			lMaterial->TransparencyFactor.Set(0.4);
			lMaterial->ShadingModel.Set(lShadingName);
			lMaterial->Shininess.Set(0.5);
			lMaterial->Specular.Set(lBlack);
			lMaterial->SpecularFactor.Set(0.3);

			lNode->AddMaterial(lMaterial);
		}
	}

	FbxFileTexture* lTexture = FbxFileTexture::Create(pScene, "Diffuse Texture");

	// Set texture properties.
	lTexture->SetFileName("jirafe.tga"); // Resource file is in current directory.
	lTexture->SetTextureUse(FbxTexture::eStandard);
	lTexture->SetMappingType(FbxTexture::eUV);
	lTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
	lTexture->SetSwapUV(false);
	lTexture->SetTranslation(0.0, 0.0);
	lTexture->SetScale(1.0, 1.0);
	lTexture->SetRotation(0.0, 0.0);

	// don't forget to connect the texture to the corresponding property of the material
	if (lMaterial)
		lMaterial->Diffuse.ConnectSrcObject(lTexture);

	lTexture = FbxFileTexture::Create(pScene, "Ambient Texture");

	// Set texture properties.
	lTexture->SetFileName("jirafe.tga"); // Resource file is in current directory.
	lTexture->SetTextureUse(FbxTexture::eStandard);
	lTexture->SetMappingType(FbxTexture::eUV);
	lTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
	lTexture->SetSwapUV(false);
	lTexture->SetTranslation(0.0, 0.0);
	lTexture->SetScale(1.0, 1.0);
	lTexture->SetRotation(0.0, 0.0);

	// don't forget to connect the texture to the corresponding property of the material
	if (lMaterial)
		lMaterial->Ambient.ConnectSrcObject(lTexture);

	lTexture = FbxFileTexture::Create(pScene, "Emissive Texture");

	// Set texture properties.
	lTexture->SetFileName("jirafe.tga"); // Resource file is in current directory.
	lTexture->SetTextureUse(FbxTexture::eStandard);
	lTexture->SetMappingType(FbxTexture::eUV);
	lTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
	lTexture->SetSwapUV(false);
	lTexture->SetTranslation(0.0, 0.0);
	lTexture->SetScale(1.0, 1.0);
	lTexture->SetRotation(0.0, 0.0);

	// don't forget to connect the texture to the corresponding property of the material
	if (lMaterial)
	{
		lMaterial->Emissive.ConnectSrcObject(lTexture);
	}
}

FbxMesh* CreateCubeWithTexture_____(FbxScene* pScene, const char* pName)
{
	FbxMesh* lMesh = FbxMesh::Create(pScene, pName);

	FbxVector4 vertex0(-50, 0, 50);
	FbxVector4 vertex1(50, 0, 50);
	FbxVector4 vertex2(50, 100, 50);
	FbxVector4 vertex3(-50, 100, 50);
	FbxVector4 vertex4(-50, 0, -50);
	FbxVector4 vertex5(50, 0, -50);
	FbxVector4 vertex6(50, 100, -50);
	FbxVector4 vertex7(-50, 100, -50);

	FbxVector4 lNormalXPos(1, 0, 0);
	FbxVector4 lNormalXNeg(-1, 0, 0);
	FbxVector4 lNormalYPos(0, 1, 0);
	FbxVector4 lNormalYNeg(0, -1, 0);
	FbxVector4 lNormalZPos(0, 0, 1);
	FbxVector4 lNormalZNeg(0, 0, -1);

	// Create control points.
	lMesh->InitControlPoints(24);
	FbxVector4* lControlPoints = lMesh->GetControlPoints();

	lControlPoints[0] = vertex0;
	lControlPoints[1] = vertex1;
	lControlPoints[2] = vertex2;
	lControlPoints[3] = vertex3;
	lControlPoints[4] = vertex1;
	lControlPoints[5] = vertex5;
	lControlPoints[6] = vertex6;
	lControlPoints[7] = vertex2;
	lControlPoints[8] = vertex5;
	lControlPoints[9] = vertex4;
	lControlPoints[10] = vertex7;
	lControlPoints[11] = vertex6;
	lControlPoints[12] = vertex4;
	lControlPoints[13] = vertex0;
	lControlPoints[14] = vertex3;
	lControlPoints[15] = vertex7;
	lControlPoints[16] = vertex3;
	lControlPoints[17] = vertex2;
	lControlPoints[18] = vertex6;
	lControlPoints[19] = vertex7;
	lControlPoints[20] = vertex1;
	lControlPoints[21] = vertex0;
	lControlPoints[22] = vertex4;
	lControlPoints[23] = vertex5;


	// We want to have one normal for each vertex (or control point),
	// so we set the mapping mode to eBY_CONTROL_POINT.
	FbxGeometryElementNormal* lGeometryElementNormal = lMesh->CreateElementNormal();
										   
	lGeometryElementNormal->SetMappingMode(FbxGeometryElement::eByControlPoint);

	// Here are two different ways to set the normal values.
	bool firstWayNormalCalculations = true;
	if (firstWayNormalCalculations)
	{
		// The first method is to set the actual normal value
		// for every control point.
		lGeometryElementNormal->SetReferenceMode(FbxGeometryElement::eDirect);

		lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);
	}
	else
	{
		// The second method is to the possible values of the normals
		// in the direct array, and set the index of that value
		// in the index array for every control point.
		lGeometryElementNormal->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

		// Add the 6 different normals to the direct array
		lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
		lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);

		// Now for each control point, we need to specify which normal to use
		lGeometryElementNormal->GetIndexArray().Add(0); // index of lNormalZPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(0); // index of lNormalZPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(0); // index of lNormalZPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(0); // index of lNormalZPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(1); // index of lNormalXPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(1); // index of lNormalXPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(1); // index of lNormalXPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(1); // index of lNormalXPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(2); // index of lNormalZNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(2); // index of lNormalZNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(2); // index of lNormalZNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(2); // index of lNormalZNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(3); // index of lNormalXNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(3); // index of lNormalXNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(3); // index of lNormalXNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(3); // index of lNormalXNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(4); // index of lNormalYPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(4); // index of lNormalYPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(4); // index of lNormalYPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(4); // index of lNormalYPos in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(5); // index of lNormalYNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(5); // index of lNormalYNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(5); // index of lNormalYNeg in the direct array.
		lGeometryElementNormal->GetIndexArray().Add(5); // index of lNormalYNeg in the direct array.
	}

	// Array of polygon vertices.
	int lPolygonVertices[] = { 
		0, 1, 2, 3,
		4, 5, 6, 7,
		8, 9, 10, 11,
		12, 13, 14, 15,
		16, 17, 18, 19,
		20, 21, 22, 23 };

	// Create UV for Diffuse channel
	FbxGeometryElementUV* lUVDiffuseElement = lMesh->CreateElementUV("DiffuseUV");
	lUVDiffuseElement->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
	lUVDiffuseElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

	FbxVector2 lVectors0(0, 0);
	FbxVector2 lVectors1(1, 0);
	FbxVector2 lVectors2(1, 1);
	FbxVector2 lVectors3(0, 1);

	lUVDiffuseElement->GetDirectArray().Add(lVectors0);
	lUVDiffuseElement->GetDirectArray().Add(lVectors1);
	lUVDiffuseElement->GetDirectArray().Add(lVectors2);
	lUVDiffuseElement->GetDirectArray().Add(lVectors3);


	// Create UV for Ambient channel
	FbxGeometryElementUV* lUVAmbientElement = lMesh->CreateElementUV("AmbientUV");

	lUVAmbientElement->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
	lUVAmbientElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

	lVectors0.Set(0, 0);
	lVectors1.Set(1, 0);
	lVectors2.Set(0, 0.418586879968643);
	lVectors3.Set(1, 0.418586879968643);

	lUVAmbientElement->GetDirectArray().Add(lVectors0);
	lUVAmbientElement->GetDirectArray().Add(lVectors1);
	lUVAmbientElement->GetDirectArray().Add(lVectors2);
	lUVAmbientElement->GetDirectArray().Add(lVectors3);

	// Create UV for Emissive channel
	FbxGeometryElementUV* lUVEmissiveElement = lMesh->CreateElementUV("EmissiveUV");

	lUVEmissiveElement->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
	lUVEmissiveElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

	lVectors0.Set(0.2343, 0);
	lVectors1.Set(1, 0.555);
	lVectors2.Set(0.333, 0.999);
	lVectors3.Set(0.555, 0.666);

	lUVEmissiveElement->GetDirectArray().Add(lVectors0);
	lUVEmissiveElement->GetDirectArray().Add(lVectors1);
	lUVEmissiveElement->GetDirectArray().Add(lVectors2);
	lUVEmissiveElement->GetDirectArray().Add(lVectors3);

	//Now we have set the UVs as eINDEX_TO_DIRECT reference and in eBY_POLYGON_VERTEX  mapping mode
	//we must update the size of the index array.
	lUVDiffuseElement->GetIndexArray().SetCount(24);
	lUVAmbientElement->GetIndexArray().SetCount(24);
	lUVEmissiveElement->GetIndexArray().SetCount(24);


	int i, j;
	// Create polygons. Assign texture and texture UV indices.
	for (i = 0; i < 6; i++)
	{
		//we won't use the default way of assigning textures, as we have
		//textures on more than just the default (diffuse) channel.
		lMesh->BeginPolygon(-1, -1, false);

		for (j = 0; j < 4; j++)
		{
			//this function points 
			lMesh->AddPolygon(lPolygonVertices[i * 4 + j]); // Control point index. 
			
			//Now we have to update the index array of the UVs for diffuse, ambient and emissive
			lUVDiffuseElement->GetIndexArray().SetAt(i * 4 + j, j);
			lUVAmbientElement->GetIndexArray().SetAt(i * 4 + j, j);
			lUVEmissiveElement->GetIndexArray().SetAt(i * 4 + j, j);
		}

		lMesh->EndPolygon();
	}

	CreateTexture_____(pScene, lMesh);
	//return lNode;
	return lMesh;
}

FbxMesh* CreateGeometry(const exportMeshNode* const node, FbxScene* const fbxScene)
{

#if 1
	FbxMesh* fbxMesh = CreateCubeWithTexture_____(fbxScene, node->m_name.c_str());
	return fbxMesh;
#else

	const std::shared_ptr<exportMesh>& mesh = node->m_mesh;
	FbxMesh* const fbxMesh = FbxMesh::Create(fbxScene, node->m_name.c_str());

	fbxMesh->InitControlPoints(int(mesh->m_positions.size()));
	FbxVector4* const vertex = fbxMesh->GetControlPoints();
	for (int i = 0; i < int(mesh->m_positions.size()); ++i)
	{
		vertex[i].Set(mesh->m_positions[i].m_x, mesh->m_positions[i].m_y, mesh->m_positions[i].m_z);
	}
	for (int i = 0; i < int(mesh->m_materials.size()); ++i)
	{
		const exportMaterial& material = mesh->m_materials[i];

		FbxGeometryElementMaterial* const fbxMaterialElement = fbxMesh->CreateElementMaterial();
		fbxMaterialElement->SetMappingMode(FbxGeometryElement::eAllSame);
		fbxMaterialElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);
		fbxMaterialElement->GetIndexArray().Add(i);

		int acc = 0;
		for (int j = 0; j < int(material.m_faceIndexCount.size()); ++j)
		{
			int vCount = material.m_faceIndexCount[i];
			fbxMesh->BeginPolygon(i);
			for (int k = 0; k < vCount; k++)
			{
				int index = material.m_indexArray[acc + k];
				int controlPointIndex = mesh->m_positionsIndex[index];
				fbxMesh->AddPolygon(controlPointIndex);
			}
			fbxMesh->EndPolygon();

			acc += vCount;
		}
	}

	// specify normals per polygon vertex.
	FbxGeometryElementNormal* const normalElement = fbxMesh->CreateElementNormal();
	normalElement->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
	normalElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);
	for (int i = 0; i < int(mesh->m_normals.size()); ++i)
	{
		normalElement->GetDirectArray().Add(FbxVector4(mesh->m_normals[i].m_x, mesh->m_normals[i].m_y, mesh->m_normals[i].m_z));
	}
	for (int i = 0; i < int(mesh->m_materials.size()); ++i)
	{
		const exportMaterial& material = mesh->m_materials[i];
		int acc = 0;
		for (int j = 0; j < int(material.m_faceIndexCount.size()); ++j)
		{
			int vCount = material.m_faceIndexCount[i];
			for (int k = 0; k < vCount; k++)
			{
				int index = material.m_indexArray[acc + k];
				int normalPointIndex = mesh->m_normalsIndex[index];
				normalElement->GetIndexArray().Add(normalPointIndex);
			}
			acc += vCount;
		}
	}

	// specify uv per polygon vertex.
	FbxGeometryElementUV* const uvElement = fbxMesh->CreateElementUV("UVSet1");
	uvElement->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
	uvElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);
	for (int i = 0; i < int(mesh->m_uvs.size()); ++i)
	{
		uvElement->GetDirectArray().Add(FbxVector2(mesh->m_uvs[i].m_x, mesh->m_uvs[i].m_y));
	}
	for (int i = 0; i < int(mesh->m_materials.size()); ++i)
	{
		const exportMaterial& material = mesh->m_materials[i];
		int acc = 0;
		for (int j = 0; j < int(material.m_faceIndexCount.size()); ++j)
		{
			int vCount = material.m_faceIndexCount[i];
			for (int k = 0; k < vCount; k++)
			{
				int index = material.m_indexArray[acc + k];
				int uvPointIndex = mesh->m_uvIndex[index];
				uvElement->GetIndexArray().Add(uvPointIndex);
			}
			acc += vCount;
		}
	}

#endif

	return fbxMesh;
}

void CreateGeometries(const exportMeshNode* const model, FbxScene* const fbxScene)
{
	int stack = 1;
	const exportMeshNode* nodeArray[256];

	nodeArray[0] = model;
	while (stack)
	{
		stack--;

		const exportMeshNode* const node = nodeArray[stack];
		if (node->m_mesh)
		{
			FbxNode* const fbxNode = node->m_fbxNode;
			FbxMesh* const mesh = CreateGeometry(node, fbxScene);
			fbxNode->SetNodeAttribute(mesh);
			fbxNode->SetShadingMode(FbxNode::eTextureShading);
		}

		for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
			iter != node->m_children.end(); iter++)
		{
			nodeArray[stack] = *iter;
			stack++;
		}
	}
}

void AnimateSkeleton(const exportMeshNode* const model, FbxScene* const fbxScene, FbxNode* const fbxModelRoot)
{
	FbxAnimStack* const animStack = FbxAnimStack::Create(fbxScene, "animStack");
	FbxAnimLayer* const animLayer = FbxAnimLayer::Create(fbxScene, "baseLayer");	// the AnimLayer object name is "Base Layer"
	animStack->AddMember(animLayer);

	double fps = 1.0f / 30.0f;
	const exportMeshNode* nodePool[256];
	//int stack = 1;
	//nodePool[0] = model;

	int stack = 0;
	for (std::list<exportMeshNode*>::const_iterator iter = model->m_children.begin();
		iter != model->m_children.end(); iter++)
	{
		nodePool[stack] = *iter;
		stack++;
	}
	
	while (stack) 
	{
		stack--;
		const exportMeshNode* const node = nodePool[stack];

		//if (node->m_keyFrame.size())
		//{
		//	FbxNode* const fbxNode = node->m_fbxNode;
		//	fbxNode->LclTranslation.GetCurveNode(animLayer, true);
		//	FbxAnimCurve* const curvePositX = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, true);
		//	FbxAnimCurve* const curvePositY = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, true);
		//	FbxAnimCurve* const curvePositZ = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, true);
		//	FbxAnimCurve* const curveRotationX = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, true);
		//	FbxAnimCurve* const curveRotationY = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, true);
		//	FbxAnimCurve* const curveRotationZ = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, true);
		//
		//	FbxTime lTime;
		//	curvePositX->KeyModifyBegin();
		//	curvePositY->KeyModifyBegin();
		//	curvePositZ->KeyModifyBegin();
		//	curveRotationX->KeyModifyBegin();
		//	curveRotationY->KeyModifyBegin();
		//	curveRotationZ->KeyModifyBegin();
		//
		//	exportVector euler1;
		//	exportVector eulerRef(node->m_keyFrame[0].CalcPitchYawRoll(euler1));
		//	for (int i = 0; i < node->m_keyFrame.size(); ++i)
		//	{
		//		double time = double(i) * fps;
		//		lTime.SetSecondDouble(time);
		//	
		//		exportVector posit(node->m_keyFrame[i].m_posit);
		//	
		//		//if (node->m_name == "Hips")
		//		//{
		//		//	//posit.m_x = 0;
		//		//	posit.m_z = 0;
		//		//}
		//	
		//		int keyIndexPositX = curvePositX->KeyAdd(lTime);
		//		curvePositX->KeySetValue(keyIndexPositX, posit.m_x);
		//		curvePositX->KeySetInterpolation(keyIndexPositX, FbxAnimCurveDef::eInterpolationCubic);
		//	
		//		int keyIndexPositY = curvePositY->KeyAdd(lTime);
		//		curvePositY->KeySetValue(keyIndexPositY, posit.m_y);
		//		curvePositY->KeySetInterpolation(keyIndexPositY, FbxAnimCurveDef::eInterpolationCubic);
		//	
		//		int keyIndexPositZ = curvePositZ->KeyAdd(lTime);
		//		curvePositZ->KeySetValue(keyIndexPositZ, posit.m_z);
		//		curvePositZ->KeySetInterpolation(keyIndexPositZ, FbxAnimCurveDef::eInterpolationCubic);
		//
		//		exportVector euler0 (node->m_keyFrame[i].CalcPitchYawRoll(euler1));
		//		float angleError = node->CalculateDeltaAngle(euler0.m_z, eulerRef.m_z);
		//		if (fabsf(angleError) > 90.0 * M_PI / 180.0f)
		//		{
		//			euler0 = euler1;
		//		}
		//		float deltax = node->CalculateDeltaAngle(euler0.m_x, eulerRef.m_x);
		//		float deltay = node->CalculateDeltaAngle(euler0.m_y, eulerRef.m_y);
		//		float deltaz = node->CalculateDeltaAngle(euler0.m_z, eulerRef.m_z);
		//
		//		eulerRef.m_x += deltax;
		//		eulerRef.m_y += deltay;
		//		eulerRef.m_z += deltaz;
		//		exportVector eulers(eulerRef.Scale(180.0f / M_PI));
		//		
		//		int keyIndexRotationX = curveRotationX->KeyAdd(lTime);
		//		curveRotationX->KeySetValue(keyIndexRotationX, eulers.m_x);
		//		curveRotationX->KeySetInterpolation(keyIndexRotationX, FbxAnimCurveDef::eInterpolationCubic);
		//		
		//		int keyIndexRotationY = curveRotationY->KeyAdd(lTime);
		//		curveRotationY->KeySetValue(keyIndexRotationY, eulers.m_y);
		//		curveRotationY->KeySetInterpolation(keyIndexRotationY, FbxAnimCurveDef::eInterpolationCubic);
		//		
		//		int keyIndexRotationZ = curveRotationZ->KeyAdd(lTime);
		//		curveRotationZ->KeySetValue(keyIndexRotationZ, eulers.m_z);
		//		curveRotationZ->KeySetInterpolation(keyIndexRotationZ, FbxAnimCurveDef::eInterpolationCubic);
		//	}
		//
		//	curvePositX->KeyModifyEnd();
		//	curvePositY->KeyModifyEnd();
		//	curvePositZ->KeyModifyEnd();
		//	curveRotationX->KeyModifyEnd();
		//	curveRotationY->KeyModifyEnd();
		//	curveRotationZ->KeyModifyEnd();
		//}

		for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
			iter != node->m_children.end(); iter++)
		{
			nodePool[stack] = *iter;
			stack++;
		}
	}
}

bool CreateScene(const exportMeshNode* const model, FbxManager* const sdkManager, FbxScene* const fbxScene)
{
	// create sbxScene info
	FbxDocumentInfo* const sceneInfo = FbxDocumentInfo::Create(sdkManager, "SceneInfo");
	sceneInfo->mTitle = "";
	sceneInfo->mSubject = "";
	sceneInfo->mAuthor = "Newton Dynamics";
	sceneInfo->mRevision = "rev. 1.0";
	sceneInfo->mKeywords = "";
	sceneInfo->mComment = "";

	// we need to add the sceneInfo before calling AddThumbNailToScene because
	// that function is asking the sbxScene for the sceneInfo.
	fbxScene->SetSceneInfo(sceneInfo);

	FbxNode* const fbxSceneRootNode = fbxScene->GetRootNode();
	FbxNode* const fbxModelRootNode = CreateSkeleton(model, fbxScene);
	CreateGeometries(model, fbxScene);
	fbxSceneRootNode->AddChild(fbxModelRootNode);

	//// Store poses
	//LinkPatchToSkeleton(sbxScene, lPatch, fbxModelRootNode);
	//StoreBindPose(sbxScene, lPatch);
	//StoreRestPose(sbxScene, fbxModelRootNode);
	
	// Animation
	AnimateSkeleton(model, fbxScene, fbxModelRootNode);

	return true;
}
