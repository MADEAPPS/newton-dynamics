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
#include <ndNewtonInc.h>

int main(int argc, char** argv)
{
	const char* name = nullptr;
	if ((argc > 1) && strstr(argv[1], ".fbx"))
	{
		name = argv[1];
	}
	if (!name)
	{
		printf("usage fbx2ndMesh [fbx_file_name]\n");
		return 0;
	}

	ndString fbxPath(name);
	ndString path(fbxPath);
	path.ToLower();
	ndString ndPath(fbxPath.SubString(0, path.Find(".fbx")) + ".nd");

	ndMeshLoader loader;
	loader.ImportFbx(fbxPath);
	loader.SaveMesh(ndPath);
	return 0;
}

