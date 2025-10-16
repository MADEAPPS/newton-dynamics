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
#include <ndNewton.h>

ndString ndGetWorkingFileName(const char* const name)
{
	char outPathName[2048];
	//ndGetWorkingFileName(name, outPathName);

	char appPath[256];
	GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
	strtolwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	snprintf(outPathName, sizeof(appPath), "%sapplications/media/%s", appPath, name);

	return ndString(outPathName);
}

int main(int argc, char** argv)
{
	const char* ndmName = nullptr;
	if ((argc > 1) && strstr(argv[1], ".fbx"))
	{
		ndmName = argv[1];
	}
	if (!ndmName)
	{
		printf("usage fbx2ndMesh [fbx_file_name]\n");
		//printf("nd_file_name = nd file name\n");
		//return 0;
	}

	ndString fullPath(ndGetWorkingFileName("playground.fbx"));

	//exportMeshNode* const mesh = exportMeshNode::ImportNdm(ndmName);
	//
	//if (!mesh)
	//{
	//	printf("failed to load %s\n", ndmName);
	//	return 0;
	//}
	//
	////const char* const exportName = acmName ? acmName : asfName;
	//if (!ExportFbx(mesh, ndmName))
	//{
	//	printf("failed to export %s\n", ndmName);
	//}
	//
	//delete mesh;
	return 0;
}

