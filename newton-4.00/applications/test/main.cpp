/* Copyright (c) <2003-2019> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "testStdafx.h"

int main (int argc, const char * argv[]) 
{
	dNewton newton;

	newton.SetThreadCount(4);
	newton.SetSubSteps(2);

	for (int i = 0; i < 10000; i ++)
	{
		newton.Update(1.0f / 60.0f);
		//newton.Sync();
	}

	return 0;
}

