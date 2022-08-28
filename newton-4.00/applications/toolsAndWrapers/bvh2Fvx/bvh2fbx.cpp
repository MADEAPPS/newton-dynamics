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

// FbxtoNgd.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "BvhNode.h"

#ifdef IOS_REF
	#undef  IOS_REF
	#define IOS_REF (*(pManager->GetIOSettings()))
#endif

static int InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene);
static FbxNode* CreateSkeleton(BvhNode* const model, FbxScene* pScene);
static bool CreateScene(BvhNode* const model, FbxManager *pSdkManager, FbxScene* pScene);
static bool SaveScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename, int pFileFormat = -1, bool pEmbedMedia = false);

int InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene)
{
	//The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
	pManager = FbxManager::Create();
	if (!pManager)
	{
		FBXSDK_printf("Error: Unable to create FBX Manager!\n");
		exit(1);
	}
	else FBXSDK_printf("Autodesk FBX SDK version %s\n", pManager->GetVersion());

	//Create an IOSettings object. This object holds all import/export settings.
	FbxIOSettings* ios = FbxIOSettings::Create(pManager, IOSROOT);
	pManager->SetIOSettings(ios);

	//Load plugins from the executable directory (optional)
	FbxString lPath = FbxGetApplicationDirectory();
	pManager->LoadPluginsDirectory(lPath.Buffer());

	//Create an FBX scene. This object holds most objects imported/exported from/to files.
	pScene = FbxScene::Create(pManager, "My Scene");
	if (!pScene)
	{
		FBXSDK_printf("Error: Unable to create FBX scene!\n");
		return 0;
	}
	return 1;
}

bool SaveScene(FbxManager* pManager, FbxDocument* pScene, const char* name, int pFileFormat, bool pEmbedMedia)
{
	int lMajor, lMinor, lRevision;
	bool lStatus = true;

	char pFilename[256];
	sprintf(pFilename, "%s", name);
	_strlwr(pFilename);
	char* ptr = strrchr(pFilename, '.');
	if (ptr && !strcmp(ptr, ".bvh"))
	{
		*ptr = 0;
	}
	strcat(pFilename, ".fbx");

	// Create an exporter.
	FbxExporter* lExporter = FbxExporter::Create(pManager, "");

	if (pFileFormat < 0 || pFileFormat >= pManager->GetIOPluginRegistry()->GetWriterFormatCount())
	{
		// Write in fall back format in less no ASCII format found
		pFileFormat = pManager->GetIOPluginRegistry()->GetNativeWriterFormat();

		//Try to export in ASCII if possible
		int	lFormatCount = pManager->GetIOPluginRegistry()->GetWriterFormatCount();

		for (int lFormatIndex = 0; lFormatIndex < lFormatCount; lFormatIndex++)
		{
			if (pManager->GetIOPluginRegistry()->WriterIsFBX(lFormatIndex))
			{
				FbxString lDesc = pManager->GetIOPluginRegistry()->GetWriterFormatDescription(lFormatIndex);
				//const char *lASCII = "ascii";
				const char *lASCII = "binary";
				if (lDesc.Find(lASCII) >= 0)
				{
					pFileFormat = lFormatIndex;
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
	IOS_REF.SetBoolProp(EXP_FBX_EMBEDDED, pEmbedMedia);
	IOS_REF.SetBoolProp(EXP_FBX_SHAPE, true);
	IOS_REF.SetBoolProp(EXP_FBX_GOBO, true);
	IOS_REF.SetBoolProp(EXP_FBX_ANIMATION, true);
	IOS_REF.SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);

	// Initialize the exporter by providing a filename.
	if (lExporter->Initialize(pFilename, pFileFormat, pManager->GetIOSettings()) == false)
	{
		FBXSDK_printf("Call to FbxExporter::Initialize() failed.\n");
		FBXSDK_printf("Error returned: %s\n\n", lExporter->GetStatus().GetErrorString());
		return false;
	}

	FbxManager::GetFileFormatVersion(lMajor, lMinor, lRevision);
	FBXSDK_printf("FBX file format version %d.%d.%d\n\n", lMajor, lMinor, lRevision);

	// Export the scene.
	lStatus = lExporter->Export(pScene);

	// Destroy the exporter.
	lExporter->Destroy();
	return lStatus;
}

int main(int argc, char** argv)
{
	bool exportSkeleton = true;
	bool exportAnimations = true;
	const char* name = nullptr;
	for (int i = 1; i < argc; i++) 
	{
		if (argv[i][0] == '-') 
		{
			if (argv[i][1] == 'm') 
			{
				exportSkeleton = true;
				exportAnimations = false;
			}
			else if (argv[i][1] == 'a') 
			{
				exportSkeleton = false;
				exportAnimations = true;
			}
		}
		else 
		{
			name = argv[i];
		}
	}
	
	if (!name) 
	{
		printf("bvh2fbx [bvh_file_name] -m -a\n");
		printf("[bvh_file_name] = bvh file name\n");
		printf("-m = export mesh only\n");
		printf("-a = export animation only\n");
		return 0;
	}

	BvhNode* const bvhSkeleton = BvhNode::LoadBvhSkeleton(name);
	if (!bvhSkeleton)
	{
		printf("failed to load %s\n", name);
		return 0;
	}
	
	FbxScene* fbxScene = nullptr;
	FbxManager* fbxManager = nullptr;
	if (!InitializeSdkObjects(fbxManager, fbxScene)) 
	{
		FBXSDK_printf("failed to initialize fbx sdk: %s\n", argv[1]);
		delete bvhSkeleton;
		return 0;
	}

	//if (ConvertToFbx(bvhSkeleton, fbxScene, exportSkeleton, exportAnimations))
	//{
	////	if (exportAnimations) {
	////		char name[1024];
	////		strcpy(name, argv[1]);
	////		_strlwr(name);
	////		char* ptr = strstr(name, ".fbx");
	////		ptr[0] = 0;
	////		strcat(name, ".anm");
	////		ExportAnimation(name, *bvhScene);
	////	}
	////	else {
	////		char name[1024];
	////		strcpy(name, argv[1]);
	////		_strlwr(name);
	////		char* ptr = strstr(name, ".fbx");
	////		ptr[0] = 0;
	////		strcat(name, ".ngd");
	////		bvhScene->Serialize(name);
	////	}
	//}

	// Create the scene.
	bool lResult = CreateScene(bvhSkeleton, fbxManager, fbxScene);

	if (lResult == false)
	{
		FBXSDK_printf("\n\nAn error occurred while creating the scene...\n");
		delete bvhSkeleton;
		fbxManager->Destroy();
		return 0;
	}

	// Save the scene.
	lResult = SaveScene(fbxManager, fbxScene, name);
	if (lResult == false)
	{
		FBXSDK_printf("\n\nAn error occurred while saving the scene...\n");
		delete bvhSkeleton;
		fbxManager->Destroy();
		return 0;
	}

	delete bvhSkeleton;
	fbxManager->Destroy();
	return 0;
}

bool CreateScene(BvhNode* const model, FbxManager *pSdkManager, FbxScene* pScene)
{
	// create scene info
	FbxDocumentInfo* sceneInfo = FbxDocumentInfo::Create(pSdkManager, "SceneInfo");
	sceneInfo->mTitle = "motion capture skeleton";
	sceneInfo->mSubject = "convert motion capture skeleton";
	sceneInfo->mAuthor = "Newton Dynamics";
	sceneInfo->mRevision = "rev. 1.0";
	sceneInfo->mKeywords = "";
	sceneInfo->mComment = "";

	// we need to add the sceneInfo before calling AddThumbNailToScene because
	// that function is asking the scene for the sceneInfo.
	pScene->SetSceneInfo(sceneInfo);

	//FbxNode* lPatch = CreatePatch(pScene, "Patch");
	FbxNode* lSkeletonRoot = CreateSkeleton(model, pScene);

	// Build the node tree.
	FbxNode* lRootNode = pScene->GetRootNode();
	//lRootNode->AddChild(lPatch);
	lRootNode->AddChild(lSkeletonRoot);

	//// Store poses
	//LinkPatchToSkeleton(pScene, lPatch, lSkeletonRoot);
	//StoreBindPose(pScene, lPatch);
	//StoreRestPose(pScene, lSkeletonRoot);
	//
	//// Animation
	//AnimateSkeleton(pScene, lSkeletonRoot);

	return true;
}

FbxNode* CreateSkeleton(BvhNode* const model, FbxScene* pScene)
{
	int stack = 1;
	BvhNode* bvhNodePool[256];
	FbxNode* fbxNodesParent[256];
	bvhNodePool[0] = model;
	fbxNodesParent[0] = nullptr;

	FbxNode* skeleton = nullptr;
	while (stack)
	{
		stack--;
		BvhNode* const bvhNode = bvhNodePool[stack];
		FbxNode* const fbxParent = fbxNodesParent[stack];

		FbxNode* const fbxNode = FbxNode::Create(pScene, bvhNode->m_name.c_str());
		bvhVector posit(bvhNode->m_matrix.m_posit);
		fbxNode->LclTranslation.Set(FbxVector4(posit.m_x, posit.m_y, posit.m_z));

		FbxSkeleton* const attribute = FbxSkeleton::Create(pScene, model->m_name.c_str());
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

		for (std::list<BvhNode*>::const_iterator iter = bvhNode->m_children.begin();
			iter != bvhNode->m_children.end(); iter++)
		{
			bvhNodePool[stack] = *iter;
			fbxNodesParent[stack] = fbxNode;
			stack++;
		}
	}

	return skeleton;
}
