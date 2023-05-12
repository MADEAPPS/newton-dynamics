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
#include "ExportFbx.h"
#include "exportMeshNode.h"

#ifdef IOS_REF
	#undef  IOS_REF
	#define IOS_REF (*(fbsManager->GetIOSettings()))
#endif

static int InitializeSdkObjects(FbxManager*& fbxManager, FbxScene*& fbxScene);
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
		//exportVector posit(node->m_matrix.m_posit);
		//exportVector euler(node->m_globalEuler.Scale(180.0f / M_PI));
		exportVector posit(node->m_localMatrix.m_posit);
		exportVector euler(node->m_localMatrix.CalcPitchYawRoll().Scale(180.0f / M_PI));
		node->m_fbxNode = fbxNode;

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

static void AnimateSkeleton(const exportMeshNode* const model, FbxScene* const fbxScene, FbxNode* const fbxModelRoot)
{
	FbxAnimStack* const animStack = FbxAnimStack::Create(fbxScene, "animStack");
	FbxAnimLayer* const animLayer = FbxAnimLayer::Create(fbxScene, "baseLayer");	// the AnimLayer object name is "Base Layer"
	animStack->AddMember(animLayer);


	double fps = 1.0f / 60.0f;
	int stack = 1;
	const exportMeshNode* nodePool[256];
	nodePool[0] = model;
	while (stack) 
	{
		stack--;
		const exportMeshNode* const root = nodePool[stack];

//if (root->m_name == "root")

		if (root->m_positionsKeys.size())
		{
			FbxNode* const fbxNode = root->m_fbxNode;
			fbxNode->LclTranslation.GetCurveNode(animLayer, true);
			FbxAnimCurve* const curveX = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, true);
			FbxAnimCurve* const curveY = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, true);
			FbxAnimCurve* const curveZ = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, true);
			
			FbxTime lTime;
			curveX->KeyModifyBegin();
			curveY->KeyModifyBegin();
			curveZ->KeyModifyBegin();
			for (int i = 0; i < root->m_positionsKeys.size(); ++i)
			{
				double time = double(i) * fps;
				lTime.SetSecondDouble(time);
			
				exportVector posit(root->m_positionsKeys[i]);

				if (root->m_name == "root")
				{
					posit.m_x = 0;
					posit.m_z = 0;
				}

				int keyIndexX = curveX->KeyAdd(lTime);
				curveX->KeySetValue(keyIndexX, posit.m_x);
				curveX->KeySetInterpolation(keyIndexX, FbxAnimCurveDef::eInterpolationCubic);
			
				int keyIndexY = curveY->KeyAdd(lTime);
				curveY->KeySetValue(keyIndexY, posit.m_y);
				curveY->KeySetInterpolation(keyIndexY, FbxAnimCurveDef::eInterpolationCubic);
			
				int keyIndexZ = curveZ->KeyAdd(lTime);
				curveZ->KeySetValue(keyIndexZ, posit.m_z);
				curveZ->KeySetInterpolation(keyIndexZ, FbxAnimCurveDef::eInterpolationCubic);
			}
			curveX->KeyModifyEnd();
			curveY->KeyModifyEnd();
			curveZ->KeyModifyEnd();
		}

//if ((root->m_name == "lowerback") || 
//	(root->m_name == "thorax") ||
//	(root->m_name == "lowerneck") ||
//	(root->m_name == "upperneck") ||
//	(root->m_name == "head") ||
//	(root->m_name == "rclavicle") ||
//	(root->m_name == "rhumerus") ||
//	//(root->m_name == "rradius") ||
//	//(root->m_name == "rwrist") ||
//	//(root->m_name == "rhand") ||
//	//(root->m_name == "rfingers") ||
//	//(root->m_name == "rthumb") ||
//
//	(root->m_name == "upperback"))
//if (root->m_name == "rhumerus")

		if (root->m_rotationsKeys.size())
		{
			FbxNode* const fbxNode = root->m_fbxNode;
			fbxNode->LclRotation.GetCurveNode(animLayer, true);
			FbxAnimCurve* const curveX = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, true);
			FbxAnimCurve* const curveY = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, true);
			FbxAnimCurve* const curveZ = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, true);

			FbxTime lTime;
			curveX->KeyModifyBegin();
			curveY->KeyModifyBegin();
			curveZ->KeyModifyBegin();
			for (int i = 0; i < root->m_rotationsKeys.size(); ++i)
			{
				double time = double(i) * fps;
				lTime.SetSecondDouble(time);

				exportVector eulers(root->m_rotationsKeys[i]);

				eulers.m_x = root->m_rotationsKeys[i].m_x;
				eulers.m_y = root->m_rotationsKeys[i].m_y;
				eulers.m_z = root->m_rotationsKeys[i].m_z;
				eulers = eulers.Scale(180.0f / M_PI);

				int keyIndexX = curveX->KeyAdd(lTime);
				curveX->KeySetValue(keyIndexX, eulers.m_x);
				curveX->KeySetInterpolation(keyIndexX, FbxAnimCurveDef::eInterpolationCubic);

				int keyIndexY = curveY->KeyAdd(lTime);
				curveY->KeySetValue(keyIndexY, eulers.m_y);
				curveY->KeySetInterpolation(keyIndexY, FbxAnimCurveDef::eInterpolationCubic);

				int keyIndexZ = curveZ->KeyAdd(lTime);
				curveZ->KeySetValue(keyIndexZ, eulers.m_z);
				curveZ->KeySetInterpolation(keyIndexZ, FbxAnimCurveDef::eInterpolationCubic);
			}
			curveX->KeyModifyEnd();
			curveY->KeyModifyEnd();
			curveZ->KeyModifyEnd();
		}

		for (std::list<exportMeshNode*>::const_iterator iter = root->m_children.begin();
			iter != root->m_children.end(); iter++)
		{
			nodePool[stack] = *iter;
			stack++;
		}
	}
}

bool CreateScene(const exportMeshNode* const model, FbxManager* const sdkManager, FbxScene* const sbxScene)
{
	// create sbxScene info
	FbxDocumentInfo* const sceneInfo = FbxDocumentInfo::Create(sdkManager, "SceneInfo");
	sceneInfo->mTitle = "motion capture skeleton";
	sceneInfo->mSubject = "convert motion capture skeleton";
	sceneInfo->mAuthor = "Newton Dynamics";
	sceneInfo->mRevision = "rev. 1.0";
	sceneInfo->mKeywords = "";
	sceneInfo->mComment = "";

	// we need to add the sceneInfo before calling AddThumbNailToScene because
	// that function is asking the sbxScene for the sceneInfo.
	sbxScene->SetSceneInfo(sceneInfo);

	//FbxNode* lPatch = CreatePatch(sbxScene, "Patch");
	FbxNode* const fbxModelRootNode = CreateSkeleton(model, sbxScene);

	FbxNode* const fbxSceneRootNode = sbxScene->GetRootNode();
	fbxSceneRootNode->AddChild(fbxModelRootNode);

	//// Store poses
	//LinkPatchToSkeleton(sbxScene, lPatch, fbxModelRootNode);
	//StoreBindPose(sbxScene, lPatch);
	//StoreRestPose(sbxScene, fbxModelRootNode);
	
	// Animation
	AnimateSkeleton(model, sbxScene, fbxModelRootNode);

	return true;
}
