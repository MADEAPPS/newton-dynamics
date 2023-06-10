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

FbxMesh* CreateGeometry(const exportMeshNode* const node, FbxScene* const fbxScene)
{
	//// indices of the vertices per each polygon
	//static int vtxId[24] = {
	//	0,1,2,3, // front  face  (Z+)
	//	1,5,6,2, // right  side  (X+)
	//	5,4,7,6, // back   face  (Z-)
	//	4,0,3,7, // left   side  (X-)
	//	0,4,5,1, // bottom face  (Y-)
	//	3,2,6,7  // top    face  (Y+)
	//};
	//
	//// control points
	//static Vector4 lControlPoints[8] = {
	//	{ -5.0,  0.0,  5.0, 1.0}, {  5.0,  0.0,  5.0, 1.0}, {  5.0,10.0,  5.0, 1.0},    { -5.0,10.0,  5.0, 1.0},
	//	{ -5.0,  0.0, -5.0, 1.0}, {  5.0,  0.0, -5.0, 1.0}, {  5.0,10.0, -5.0, 1.0},    { -5.0,10.0, -5.0, 1.0}
	//};
	//
	//// normals
	//static Vector4 lNormals[8] = {
	//	{-0.577350258827209,-0.577350258827209, 0.577350258827209, 1.0},
	//	{ 0.577350258827209,-0.577350258827209, 0.577350258827209, 1.0},
	//	{ 0.577350258827209, 0.577350258827209, 0.577350258827209, 1.0},
	//	{-0.577350258827209, 0.577350258827209, 0.577350258827209, 1.0},
	//	{-0.577350258827209,-0.577350258827209,-0.577350258827209, 1.0},
	//	{ 0.577350258827209,-0.577350258827209,-0.577350258827209, 1.0},
	//	{ 0.577350258827209, 0.577350258827209,-0.577350258827209, 1.0},
	//	{-0.577350258827209, 0.577350258827209,-0.577350258827209, 1.0}
	//};
	//
	//// uvs
	//static Vector2 lUVs[14] = {
	//	{ 0.0, 1.0},
	//	{ 1.0, 0.0},
	//	{ 0.0, 0.0},
	//	{ 1.0, 1.0}
	//};
	//
	//// indices of the uvs per each polygon
	//static int uvsId[24] = {
	//	0,1,3,2,2,3,5,4,4,5,7,6,6,7,9,8,1,10,11,3,12,0,2,13
	//};
	//
	//// create the main structure.
	//FbxMesh* fbxMesh = FbxMesh::Create(pScene, "");
	
	FbxMesh* const fbxMesh = FbxMesh::Create(fbxScene, node->m_name.c_str());
	const std::shared_ptr<exportMesh>& mesh = node->m_mesh;

	//// Create control points.
	//fbxMesh->InitControlPoints(8);
	//FbxVector4* vertex = fbxMesh->GetControlPoints();
	//memcpy((void*)vertex, (void*)lControlPoints, 8 * sizeof(FbxVector4));
	//
	//// create the materials.
	///* Each polygon face will be assigned a unique material.
	//*/
	//FbxGeometryElementMaterial* lMaterialElement = fbxMesh->CreateElementMaterial();
	//lMaterialElement->SetMappingMode(FbxGeometryElement::eAllSame);
	//lMaterialElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);
	//
	//lMaterialElement->GetIndexArray().Add(0);
	//
	//// Create polygons later after FbxGeometryElementMaterial is created. Assign material indices.
	//int vId = 0;
	//for (int f = 0; f < 6; f++)
	//{
	//	fbxMesh->BeginPolygon();
	//	for (int v = 0; v < 4; v++)
	//		fbxMesh->AddPolygon(vtxId[vId++]);
	//	fbxMesh->EndPolygon();
	//}
	//
	//// specify normals per control point.
	//FbxGeometryElementNormal* lNormalElement = fbxMesh->CreateElementNormal();
	//lNormalElement->SetMappingMode(FbxGeometryElement::eByControlPoint);
	//lNormalElement->SetReferenceMode(FbxGeometryElement::eDirect);
	//
	//for (int n = 0; n < 8; n++)
	//	lNormalElement->GetDirectArray().Add(FbxVector4(lNormals[n][0], lNormals[n][1], lNormals[n][2]));
	//
	//
	//// Create the node containing the mesh
	//FbxNode* lNode = FbxNode::Create(pScene, pName);
	//lNode->LclTranslation.Set(pLclTranslation);
	//
	//lNode->SetNodeAttribute(fbxMesh);
	//lNode->SetShadingMode(FbxNode::eTextureShading);
	//
	//// create UVset
	//FbxGeometryElementUV* lUVElement1 = fbxMesh->CreateElementUV("UVSet1");
	//FBX_ASSERT(lUVElement1 != NULL);
	//lUVElement1->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
	//lUVElement1->SetReferenceMode(FbxGeometryElement::eIndexToDirect);
	//for (int i = 0; i < 4; i++)
	//	lUVElement1->GetDirectArray().Add(FbxVector2(lUVs[i][0], lUVs[i][1]));
	//
	//for (int i = 0; i < 24; i++)
	//	lUVElement1->GetIndexArray().Add(uvsId[i % 4]);
	//
	//return lNode;

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
