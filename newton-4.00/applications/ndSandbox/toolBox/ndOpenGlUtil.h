/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
	


// OpenGlUtil.h: interface for the OpenGlUtil class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_OPENGLUTIL_H_H)
#define AFX_OPENGLUTIL_H_H

#include "ndSandboxStdafx.h"

struct dMOUSE_POINT
{
	dInt32 x;
	dInt32 y;
};

void ShowMousePicking (const dVector& p0, const dVector& p1, const dVector& originColor = dVector (1.0f, 1.0f, 0.0f, 0.0f), const dVector& lineColor =  dVector (1.0f, 1.0f, 1.0f, 0.0f)); 
void HandleResize(dInt32 width, dInt32 height);
dInt32 dGetKeyState(dInt32 key);
void GetCursorPos(dMOUSE_POINT& point);

#endif 

