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
#include "exportFbx.h"

int main(int argc, char** argv)
{
	const char* ndmName = nullptr;
	if (strstr(argv[1], ".ndm"))
	{
		ndmName = argv[1];
	}
	if (!ndmName)
	{
		printf("ndm2fbx ndm_file_name\n");
		printf("ndm_file_name = file name\n");
		return 0;
	}

	exportMeshNode* const mesh = exportMeshNode::ImportNdm(ndmName);

	if (!mesh)
	{
		printf("failed to load %s\n", ndmName);
		return 0;
	}

	//const char* const exportName = acmName ? acmName : asfName;
	if (!ExportFbx(mesh, ndmName))
	{
		printf("failed to export %s\n", ndmName);
	}

	delete mesh;
	return 0;
}

