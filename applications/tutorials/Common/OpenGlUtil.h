/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// OpenGlUtil.h: interface for the OpenGlUtil class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __OPENGLUTIL_H
#define __OPENGLUTIL_H

class dVector;


enum KEY_asciiCode
{
	KeyCode_UNKNOWN		= 0,
	KeyCode_L_BUTTON = VK_LBUTTON,
	KeyCode_R_BUTTON = VK_RBUTTON, 

	KeyCode_F1	= VK_F1,
	KeyCode_F2,
	KeyCode_F3,
	KeyCode_F4,
	KeyCode_F5,
	KeyCode_F6,
	KeyCode_F7,
	KeyCode_F8,
	KeyCode_F9,
	KeyCode_F10,
	KeyCode_F11,
	KeyCode_F12,
	KeyCode_lastSpecialKey,

	KeyCode_ESCAPE	= VK_ESCAPE ,
	KeyCode_SPACE	= VK_SPACE ,
	KeyCode_PLUS	= VK_ADD,
	KeyCode_MINUS	= VK_SUBTRACT,

	KeyCode_a = 97,
	KeyCode_b,
	KeyCode_c,
	KeyCode_d,
	KeyCode_e,
	KeyCode_f,
	KeyCode_g,
	KeyCode_h,
	KeyCode_i,
	KeyCode_j,
	KeyCode_k,
	KeyCode_l,
	KeyCode_m,
	KeyCode_n,
	KeyCode_o,
	KeyCode_p,
	KeyCode_q,
	KeyCode_r,
	KeyCode_s,
	KeyCode_t,
	KeyCode_u,
	KeyCode_v,
	KeyCode_w,
	KeyCode_x,
	KeyCode_y,
	KeyCode_z,
} ;



int InitGraphicsSystem (int width, int height);
void ProcessEvents (NewtonWorld* world);
int IsKeyDown (int keycode);
dVector GetMousePos();

void InitFont ();
void Print (const dVector& color, dFloat x, dFloat y, const char *fmt, ... );



unsigned GetTimeInMicrosenconds();
GLuint LoadTexture(const char* const filename);
void GetWorkingFileName (const char* const name, char* const outPathName);
void SetCamera (const dVector& eyePoint, const dVector& target);


dVector ScreenToWorld (const dVector& screen);
//dVector WorldToScreen (const dVector& world);
//void ShowMousePicking (const dVector& p0, const dVector& p1, dFloat radius); 

#endif 

