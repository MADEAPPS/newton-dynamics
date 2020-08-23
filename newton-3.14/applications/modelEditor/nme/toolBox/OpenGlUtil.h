/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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

#include <toolbox_stdafx.h>


//#define RECORD_LOG
//#define READ_LOG






struct dMOUSE_POINT
{
	int x;
	int y;
};


dMatrix GetModelViewMatrix ();

void HandleResize(int width, int height);
int dGetKeyState(int key);
void GetCursorPos(dMOUSE_POINT& point);
dVector ScreenToWorld (const dVector& screen);
dVector WorldToScreen (const dVector& world);
void ShowMousePicking (const dVector& p0, const dVector& p1, const dVector& originColor = dVector (1.0f, 1.0f, 0.0f, 0.0f), const dVector& lineColor =  dVector (1.0f, 1.0f, 1.0f, 0.0f)); 

void KeyUp(int key);
void KeyDown (int key);



#endif 

