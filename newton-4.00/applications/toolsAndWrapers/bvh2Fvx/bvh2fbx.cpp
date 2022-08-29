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
#include "ExportFbx.h"

#ifdef IOS_REF
	#undef  IOS_REF
	#define IOS_REF (*(manager->GetIOSettings()))
#endif


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

	BvhNode* const bvhSkeleton = BvhNode::LoadSkeleton(name);
	if (!bvhSkeleton)
	{
		printf("failed to load %s\n", name);
		return 0;
	}

	if (!ExportFbx(bvhSkeleton, name))
	{
		printf("failed to export %s\n", name);
	}
/*
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
*/
	delete bvhSkeleton;
	//fbxManager->Destroy();
	return 0;
}

