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

ndString ndGetWorkingFileName(const char* const name)
{
	ndString path(std::getenv("NewtonDynamics"));
	path += "/newton-4.00/applications/media/";
	path += name;
	return path;
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
		//return 0;
	}

	ndMeshLoader loader;
	ndString fullPath(ndGetWorkingFileName("playground.fbx"));
	
	loader.ImportFbx(fullPath);
	return 0;
}

