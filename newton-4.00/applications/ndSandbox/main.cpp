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
#include "ndLeakTrackler.h"

#include <string>
std::string xxxx0("before");
static ndSetAllocators setAllocators;
std::string xxxx1("after");

int main(int, char**)
{
	std::string xxxx2("local");
	ndDemoEntityManager demos;
	demos.Run();
	return 0;
}

