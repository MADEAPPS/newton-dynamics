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
#include "exportMeshNode.h"
#include "ExportFbx.h"

int main(int argc, char** argv)
{
#if 0
	const char* asfName = nullptr;
	const char* acmName = nullptr;
	for (int i = 1; i < argc; i++)
	{
		if (strstr(argv[i], ".asf"))
		{
			asfName = argv[i];
		}
		else if (strstr(argv[i], ".amc"))
		{
			acmName = argv[i];
		}
	}
	
	if (!asfName)
	{
		printf("asf2fbx asf_file_name [amc_file_name]\n");
		printf("asf_file_name = skeleton file name\n");
		printf("[amc_file_name] = optional animation file name\n");
		return 0;
	}
	exportMeshNode* const skeleton = exportMeshNode::ImportAsfSkeleton(asfName, acmName);

#else
	const char* bvhName = nullptr;
	if (strstr(argv[1], ".bvh"))
	{
		bvhName = argv[1];
	}
	if (!bvhName)
	{
		printf("asf2fbx bvh_file_name\n");
		printf("bvh_file_name = skeleton file name\n");
		return 0;
	}

	exportMeshNode* const skeleton = exportMeshNode::ImportBvhSkeleton(bvhName);
#endif

	if (!skeleton)
	{
		printf("failed to load %s\n", bvhName);
		return 0;
	}

	//const char* const exportName = acmName ? acmName : asfName;
	if (!ExportFbx(skeleton, bvhName))
	{
		printf("failed to export %s\n", bvhName);
	}

	delete skeleton;
	return 0;
}

