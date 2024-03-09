/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndDemoEntityManager.h"
#include "ndLeakTracker.h"

//#include <string>
//std::string xxxx0("before");

int main(int, char**)
{
	ndSetAllocators setAllocators;
	//std::string xxxx2("local0");
	//std::string xxxx3("local1");
	//for (int i = 0; i < 2000; i++)
	//{
	//	ndFloat32 x = ndGaussianRandom(1.0f, 0.4472f);
	//	ndTrace(("%g\n", x));
	//}

	ndDemoEntityManager demos;
	demos.Run();
	return 0;
}

