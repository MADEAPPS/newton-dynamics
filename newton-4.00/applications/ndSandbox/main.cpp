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

	//ndBrainVector xxx;
	//xxx.PushBack(0.1f);
	//xxx.PushBack(0.7f);
	//xxx.PushBack(0.2f);
	//
	//ndInt32 a = xxx.CategoricalSample();
	//ndInt32 b = xxx.CategoricalSample();
	//ndInt32 c = xxx.CategoricalSample();
	//ndInt32 ndBrainVector::CategoricalSample() const
	//ndFloat32 xxx0[] = { 0.31, 0.29, 0.3 };
	//ndFloat32 xxx1[3];
	//ndFloat32 xxx2[3];
	//
	//ndFloat32 a = 0.9999999f;
	////ndFloat32   a = 0.0000001f;
	//a = (a / (1.0f - a));
	//a = ndLog(a);
	//
	//xxx1[0] = ndLog(xxx0[0] / (1 - xxx0[0]));
	//xxx1[1] = ndLog(xxx0[1] / (1 - xxx0[1]));
	//xxx1[2] = ndLog(xxx0[2] / (1 - xxx0[2]));
	//
	//ndRand();
	//ndRand();
	//ndRand();
	//xxx2[0] = ndRand();
	//xxx2[1] = ndRand();
	//xxx2[2] = ndRand();
	//
	//xxx2[0] = - ndLog(-ndLog(xxx2[0]));
	//xxx2[1] = - ndLog(-ndLog(xxx2[1]));
	//xxx2[2] = - ndLog(-ndLog(xxx2[2]));
	//
	//xxx1[0] += xxx2[0];
	//xxx1[1] += xxx2[1];
	//xxx1[2] += xxx2[2];
	//
	//ndFloat32 acc = ndExp(xxx1[0]) + ndExp(xxx1[1]) + ndExp(xxx1[2]);
	//xxx1[0] = ndExp(xxx1[0]) / acc;
	//xxx1[1] = ndExp(xxx1[1]) / acc;
	//xxx1[2] = ndExp(xxx1[2]) / acc;



	ndSetAllocators setAllocators;
	//std::string xxxx2("local0");
	//std::string xxxx3("local1");

	ndDemoEntityManager demos;
	demos.Run();
	return 0;
}

