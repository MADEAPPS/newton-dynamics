/* Copyright (c) <2018-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// dTimeTracker.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "dTimeTracker.h"

/*
// This is an example of an exported variable
DTIMETRACKER_API int ndTimeTracker=0;

// This is an example of an exported function.
DTIMETRACKER_API int fndTimeTracker(void)
{
	return 42;
}

// This is the constructor of a class that has been exported.
// see dTimeTracker.h for the class definition
CdTimeTracker::CdTimeTracker()
{
	return;
}
*/


int ttOpenRecord(const char* const name)
{
	return 0;
}

void ttCloseRecord(int record)
{

}
