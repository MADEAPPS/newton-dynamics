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
	//for (ndInt32 i = 0; i < 2000; i++)
	//{
	//	ndFloat32 x = ndGaussianRandom(1.0f, 0.4472f);
	//	ndTrace(("%g\n", x));
	//}

	//ndArray<ndVector> xxxx;
	//for (int y = 0; y < 256; y++)
	//{
	//	for (int x = 0; x < 256; x++)
	//	{
	//		ndVector p(ndFloat32(x), ndFloat32(y), 0.0f, 0.0f);
	//		xxxx.PushBack(p);
	//		xxxx.PushBack(p);
	//		xxxx.PushBack(p);
	//		xxxx.PushBack(p);
	//		xxxx.PushBack(p);
	//		xxxx.PushBack(p);
	//	}
	//}
	//ndArray<ndInt32> index;
	//index.SetCount(xxxx.GetCount());
	//ndInt32 vertexCount = ndVertexListToIndexList(&xxxx[0].m_x, sizeof(ndVector), 3, ndInt32(xxxx.GetCount()), &index[0], ndFloat32(1.0e-6f));

	//ndVector points[5];
	//points[0] = ndVector(-1.0f, -1.0f, 0.0f, 0.0f);
	//points[1] = ndVector( 1.0f, -1.0f, 0.0f, 0.0f);
	//points[2] = ndVector(1.2f, 0.6f, 0.0f, 0.0f);
	//points[3] = ndVector(0.9f, 0.9f, 0.0f, 0.0f);
	//points[4] = ndVector(-1.0f, 1.0f, 0.0f, 0.0f);
	//ndArray<ndInt32> triangles;
	//ndTriangulatePolygon(points, 5, triangles);

	//ndSharedPtr<int> strong(new int);
	//ndWeakPtr<int> weak(strong);


	ndDemoEntityManager demos;
	demos.Run();
	return 0;
}

