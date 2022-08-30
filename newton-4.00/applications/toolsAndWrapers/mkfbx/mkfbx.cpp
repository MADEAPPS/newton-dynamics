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

	exportMeshNode* const skeleton = exportMeshNode::LoadSkeleton(name);
	if (!skeleton)
	{
		printf("failed to load %s\n", name);
		return 0;
	}

	if (!ExportFbx(skeleton, name))
	{
		printf("failed to export %s\n", name);
	}

	delete skeleton;
	return 0;
}

