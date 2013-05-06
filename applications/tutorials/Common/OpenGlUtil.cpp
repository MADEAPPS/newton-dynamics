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


// OpenGlUtil.cpp: implementation of the OpenGlUtil class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include <dVector.h>
#include <dMatrix.h>
#include "OpenGlUtil.h"
#include "CollectInputAndUpdateCamera.h"

// This function or variable may be unsafe. Consider using fscanf_s instead. 
// To disable deprecation, use _CRT_SECURE_NO_WARNINGS. See online help for details.
#pragma warning (disable: 4996) 
//#pragma warning (disable: 4100) //unreferenced formal parameter

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



static GLuint gAerialFontImage;
static dFloat gAerialFontUV[128][4] = 
{
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.000000f, 0.000000f},
	{0.000000f, 0.000000f, 0.019531f, 0.074219f},
	{0.023438f, 0.000000f, 0.046875f, 0.074219f},
	{0.050781f, 0.000000f, 0.082031f, 0.074219f},
	{0.085938f, 0.000000f, 0.121094f, 0.074219f},
	{0.125000f, 0.000000f, 0.160156f, 0.074219f},
	{0.164063f, 0.000000f, 0.226563f, 0.074219f},
	{0.230469f, 0.000000f, 0.277344f, 0.074219f},
	{0.281250f, 0.000000f, 0.296875f, 0.074219f},
	{0.300781f, 0.000000f, 0.324219f, 0.074219f},
	{0.328125f, 0.000000f, 0.351563f, 0.074219f},
	{0.355469f, 0.000000f, 0.382813f, 0.074219f},
	{0.386719f, 0.000000f, 0.425781f, 0.074219f},
	{0.429688f, 0.000000f, 0.449219f, 0.074219f},
	{0.453125f, 0.000000f, 0.476563f, 0.074219f},
	{0.480469f, 0.000000f, 0.500000f, 0.074219f},
	{0.503906f, 0.000000f, 0.523438f, 0.074219f},
	{0.527344f, 0.000000f, 0.562500f, 0.074219f},
	{0.566406f, 0.000000f, 0.601563f, 0.074219f},
	{0.605469f, 0.000000f, 0.640625f, 0.074219f},
	{0.644531f, 0.000000f, 0.679688f, 0.074219f},
	{0.683594f, 0.000000f, 0.718750f, 0.074219f},
	{0.722656f, 0.000000f, 0.757813f, 0.074219f},
	{0.761719f, 0.000000f, 0.796875f, 0.074219f},
	{0.800781f, 0.000000f, 0.835938f, 0.074219f},
	{0.839844f, 0.000000f, 0.875000f, 0.074219f},
	{0.878906f, 0.000000f, 0.914063f, 0.074219f},
	{0.917969f, 0.000000f, 0.941406f, 0.074219f},
	{0.945313f, 0.000000f, 0.968750f, 0.074219f},
	{0.000000f, 0.078125f, 0.039063f, 0.152344f},
	{0.042969f, 0.078125f, 0.082031f, 0.152344f},
	{0.085938f, 0.078125f, 0.125000f, 0.152344f},
	{0.128906f, 0.078125f, 0.167969f, 0.152344f},
	{0.171875f, 0.078125f, 0.238281f, 0.152344f},
	{0.242188f, 0.078125f, 0.285156f, 0.152344f},
	{0.289063f, 0.078125f, 0.335938f, 0.152344f},
	{0.339844f, 0.078125f, 0.386719f, 0.152344f},
	{0.390625f, 0.078125f, 0.437500f, 0.152344f},
	{0.441406f, 0.078125f, 0.484375f, 0.152344f},
	{0.488281f, 0.078125f, 0.527344f, 0.152344f},
	{0.531250f, 0.078125f, 0.582031f, 0.152344f},
	{0.585938f, 0.078125f, 0.632813f, 0.152344f},
	{0.636719f, 0.078125f, 0.652344f, 0.152344f},
	{0.656250f, 0.078125f, 0.691406f, 0.152344f},
	{0.695313f, 0.078125f, 0.742188f, 0.152344f},
	{0.746094f, 0.078125f, 0.785156f, 0.152344f},
	{0.789063f, 0.078125f, 0.839844f, 0.152344f},
	{0.843750f, 0.078125f, 0.890625f, 0.152344f},
	{0.894531f, 0.078125f, 0.945313f, 0.152344f},
	{0.949219f, 0.078125f, 0.992188f, 0.152344f},
	{0.000000f, 0.156250f, 0.050781f, 0.230469f},
	{0.054688f, 0.156250f, 0.101563f, 0.230469f},
	{0.105469f, 0.156250f, 0.148438f, 0.230469f},
	{0.152344f, 0.156250f, 0.191406f, 0.230469f},
	{0.195313f, 0.156250f, 0.242188f, 0.230469f},
	{0.246094f, 0.156250f, 0.289063f, 0.230469f},
	{0.292969f, 0.156250f, 0.359375f, 0.230469f},
	{0.363281f, 0.156250f, 0.406250f, 0.230469f},
	{0.410156f, 0.156250f, 0.449219f, 0.230469f},
	{0.453125f, 0.156250f, 0.488281f, 0.230469f},
	{0.492188f, 0.156250f, 0.515625f, 0.230469f},
	{0.519531f, 0.156250f, 0.539063f, 0.230469f},
	{0.542969f, 0.156250f, 0.566406f, 0.230469f},
	{0.570313f, 0.156250f, 0.609375f, 0.230469f},
	{0.613281f, 0.156250f, 0.648438f, 0.230469f},
	{0.652344f, 0.156250f, 0.675781f, 0.230469f},
	{0.679688f, 0.156250f, 0.714844f, 0.230469f},
	{0.718750f, 0.156250f, 0.757813f, 0.230469f},
	{0.761719f, 0.156250f, 0.796875f, 0.230469f},
	{0.800781f, 0.156250f, 0.839844f, 0.230469f},
	{0.843750f, 0.156250f, 0.878906f, 0.230469f},
	{0.882813f, 0.156250f, 0.902344f, 0.230469f},
	{0.906250f, 0.156250f, 0.945313f, 0.230469f},
	{0.949219f, 0.156250f, 0.988281f, 0.230469f},
	{0.000000f, 0.234375f, 0.015625f, 0.308594f},
	{0.019531f, 0.234375f, 0.035156f, 0.308594f},
	{0.039063f, 0.234375f, 0.074219f, 0.308594f},
	{0.078125f, 0.234375f, 0.093750f, 0.308594f},
	{0.097656f, 0.234375f, 0.152344f, 0.308594f},
	{0.156250f, 0.234375f, 0.195313f, 0.308594f},
	{0.199219f, 0.234375f, 0.238281f, 0.308594f},
	{0.242188f, 0.234375f, 0.281250f, 0.308594f},
	{0.285156f, 0.234375f, 0.324219f, 0.308594f},
	{0.328125f, 0.234375f, 0.355469f, 0.308594f},
	{0.359375f, 0.234375f, 0.394531f, 0.308594f},
	{0.398438f, 0.234375f, 0.421875f, 0.308594f},
	{0.425781f, 0.234375f, 0.464844f, 0.308594f},
	{0.468750f, 0.234375f, 0.503906f, 0.308594f},
	{0.507813f, 0.234375f, 0.558594f, 0.308594f},
	{0.562500f, 0.234375f, 0.597656f, 0.308594f},
	{0.601563f, 0.234375f, 0.636719f, 0.308594f},
	{0.640625f, 0.234375f, 0.675781f, 0.308594f},
	{0.679688f, 0.234375f, 0.707031f, 0.308594f},
	{0.710938f, 0.234375f, 0.726563f, 0.308594f},
	{0.730469f, 0.234375f, 0.757813f, 0.308594f},
	{0.761719f, 0.234375f, 0.800781f, 0.308594f}
};											   


void InitFont ()
{
	gAerialFontImage = LoadTexture ("Arial.tga");
}



unsigned SWAP_INT32(unsigned x)
{
	return x;
}

unsigned short SWAP_INT16(unsigned short x)
{
	return x;
}

void SWAP_FLOAT32_ARRAY (void *array, int count)
{
	array;
	count;
}



GLuint LoadTexture(const char* const filename)
{
	#pragma pack(1)
	struct TGAHEADER
	{
		GLbyte	identsize;              // Size of ID field that follows header (0)
		GLbyte	colorMapType;           // 0 = None, 1 = palette
		GLbyte	imageType;              // 0 = none, 1 = indexed, 2 = rgb, 3 = grey, +8=rle
		unsigned short	colorMapStart;  // First color map entry
		unsigned short	colorMapLength; // Number of colors
		unsigned char 	colorMapBits;   // bits per palette entry
		unsigned short	xstart;         // image x origin
		unsigned short	ystart;         // image y origin
		unsigned short	width;          // width in pixels
		unsigned short	height;         // height in pixels
		GLbyte	bits;                   // bits per pixel (8 16, 24, 32)
		GLbyte	descriptor;             // image descriptor
	};
	#pragma pack(8)


	FILE *pFile;				// File pointer
	TGAHEADER tgaHeader;		// TGA file header
	unsigned lImageSize;		// Size in bytes of image
	short sDepth;				// Pixel depth;
	GLbyte	*pBits;			// Pointer to bits
	GLint iWidth;
	GLint iHeight;
	GLint iComponents;
	GLenum eFormat;
	GLuint texture;
	char fullPathName[2048];

	GetWorkingFileName (filename, fullPathName);
	
	pFile = fopen (fullPathName, "rb");
   	if(pFile == NULL) {
		return 0;
	}
	
	//_ASSERTE (sizeof (TGAHEADER) == 18);
	// Read in header (binary) sizeof(TGAHEADER) = 18
	fread(&tgaHeader, 18, 1, pFile);

	
	// Do byte swap for big vs little Indian
	tgaHeader.colorMapStart = SWAP_INT16(tgaHeader.colorMapStart);
	tgaHeader.colorMapLength = SWAP_INT16(tgaHeader.colorMapLength);
	tgaHeader.xstart = SWAP_INT16(tgaHeader.xstart);
	tgaHeader.ystart = SWAP_INT16(tgaHeader.ystart);
	tgaHeader.width = SWAP_INT16(tgaHeader.width);
	tgaHeader.height = SWAP_INT16(tgaHeader.height);
		    
	// Get width, height, and depth of texture
	iWidth = tgaHeader.width;
	iHeight = tgaHeader.height;
	sDepth = tgaHeader.bits / 8;
	_ASSERTE ((sDepth == 3) || (sDepth == 4));

	
	// Put some validity checks here. Very simply, I only understand
	// or care about 8, 24, or 32 bit targa's.
	if(tgaHeader.bits != 8 && tgaHeader.bits != 24 && tgaHeader.bits != 32) {
		fclose(pFile);
		return 0;
	}


	// Calculate size of image buffer
	lImageSize = tgaHeader.width * tgaHeader.height * sDepth;
	
	// Allocate memory and check for success
	pBits = (GLbyte *)malloc(lImageSize * sizeof(GLbyte));
	if(pBits == NULL) {
		fclose(pFile);
		return 0;
	}
	
	// Read in the bits
	// Check for read error. This should catch RLE or other 
	// weird formats that I don't want to recognize
	if(fread(pBits, lImageSize, 1, pFile) != 1)  {
		fclose(pFile);
		free(pBits);
		return 0; 
	}

	eFormat = GL_RGBA;
	//iComponents = GL_RGB8;
	iComponents = GL_RGBA;


	switch(sDepth)
	{
		// intel arch
		case 3:     // Most likely case
			eFormat = GL_BGR_EXT;
			//iComponents = GL_BGR_EXT;
			iComponents = GL_RGBA;
			break;

		case 4:
			eFormat = GL_BGRA_EXT;
			iComponents = GL_RGBA;
		break;

		case 1:
			eFormat = GL_LUMINANCE;
			iComponents = GL_LUMINANCE8;
			break;
	};



	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//	glTexImage2D(GL_TEXTURE_2D, 0, iComponents, iWidth, iHeight, 0, eFormat, GL_UNSIGNED_BYTE, pBits);

	// when texture area is small, bilinear filter the closest mipmap
//  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST );

	// when texture area is small, trilinear filter mipmaped
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	// when texture area is large, bilinear filter the first mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// build our texture mipmaps
	gluBuild2DMipmaps (GL_TEXTURE_2D, iComponents, iWidth, iHeight, eFormat, GL_UNSIGNED_BYTE, pBits);

	// Done with File
	fclose(pFile);
	free(pBits);        
	return texture;
}

void GetWorkingFileName (const char* name, char* outPathName)
{
	char path[256]; 
	GetModuleFileNameA(NULL, path, 256);

	char* const ptr = strstr (path, "tutorials");
	ptr [0] = 0;
	sprintf (outPathName, "%stutorials/bin/%s", path, name);
}


void Print (const dVector& color, dFloat x, dFloat y, const char *fmt, ... )
{
	// enter 2d display mode 
	int len;
	int witdh;
	int height;
	dFloat sx;
	dFloat sy;
	va_list argptr;
	char string[1024];

	// Note, there may be other things you need to change,
	//depending on how you have your OpenGL state set up.
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDisable (GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);

	glColor3f(color.m_x, color.m_y, color.m_z);
	// This allows alpha blending of 2D textures with the scene 
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);


	GLint viewport[4]; 
	//Retrieves the viewport and stores it in the variable
	glGetIntegerv(GL_VIEWPORT, viewport); 
	witdh = viewport[2];
	height = viewport[3];

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, (GLdouble)witdh, height, 0.0, 0.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);


	// draw the text on the screen
	glBindTexture(GL_TEXTURE_2D, gAerialFontImage);
	
	va_start (argptr, fmt);
	vsprintf (string, fmt, argptr);
	va_end( argptr );
	len = (int) strlen (string );

	glBegin(GL_QUADS);

	sx = x + 0.5f;
	sy = y + 0.5f;
	for (int i = 0; i < len; i++) {
		dFloat w;
		dFloat h;
		dFloat tx1;
		dFloat ty1;
		dFloat tx2;
		dFloat ty2;
		unsigned char c;

		c = string[i];
		tx1 = gAerialFontUV[c][0];
		ty1 = gAerialFontUV[c][1];
		tx2 = gAerialFontUV[c][2];
		ty2 = gAerialFontUV[c][3];
		w = (tx2-tx1) * 256.0f;
		h = (ty2-ty1) * 256.0f;

		glTexCoord2f (tx1, 1.0f-ty1); glVertex2f(sx + 0, sy + 0);
		glTexCoord2f (tx2, 1.0f-ty1); glVertex2f(sx + w, sy + 0);
		glTexCoord2f (tx2, 1.0f-ty2); glVertex2f(sx + w, sy + h);
		glTexCoord2f (tx1, 1.0f-ty2); glVertex2f(sx + 0, sy + h);
		sx += w;
	}
	glEnd();

	// exit 2d display mode
	glDisable(GL_BLEND);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glPopAttrib();

//	setup_opengl (screen->w, screen->h);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
}


unsigned GetTimeInMicrosenconds()
{
	unsigned ticks;
	LARGE_INTEGER count;
	static bool firstTime = true;
	static bool haveTimer = false;
	static LARGE_INTEGER frequency;
	static LARGE_INTEGER baseCount;

	if (firstTime) {
		firstTime = false;
		haveTimer = QueryPerformanceFrequency(&frequency) ? true : false;
		QueryPerformanceCounter (&baseCount);
	} 

	QueryPerformanceCounter (&count);
	count.QuadPart -= baseCount.QuadPart;
	ticks = unsigned (count.QuadPart * LONGLONG (1000000) / frequency.QuadPart);
	return ticks;
}


/*

void ShowMousePicking (const dVector& p0, const dVector& p1, dFloat radius)
{

// set up the cube's texture
glDisable (GL_LIGHTING);
glDisable(GL_TEXTURE_2D);

glColor3f(1.0f, 1.0f, 1.0f);
glLineWidth(3.0f);
glBegin(GL_LINES);
glVertex3f(p0.m_x, p0.m_y, p0.m_z);
glVertex3f(p1.m_x, p1.m_y, p1.m_z);
glEnd();
glLineWidth(1.0f);

// draw the bone points
glColor3f(1.0f, 1.0f, 0.0f);
glPointSize(6.0f);
glBegin(GL_POINTS);
glVertex3f(p0.m_x, p0.m_y, p0.m_z);
glVertex3f(p1.m_x, p1.m_y, p1.m_z);
glEnd();
glPointSize(1.0f);

glEnable (GL_LIGHTING);

//	glPopMatrix();
}

dVector WorldToScreen (const dVector& world)
{
	int win[4]; 
	GLint viewport[4]; 

	//Retrieves the viewport and stores it in the variable
	// get a point on the display arai of teh windows
	GLUI_Master.get_viewport_area(&win[0], &win[1], &win[2], &win[3]);
	viewport[0] = GLint (win[0]);
	viewport[1] = GLint (win[1]);
	viewport[2] = GLint (win[2]);
	viewport[3] = GLint (win[3]);


	//Where the 16 doubles of the matrix are to be stored
	GLdouble modelview[16]; 

	//Retrieve the matrix
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 

	GLdouble projection[16]; 
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	GLdouble winX;
	GLdouble winY;
	GLdouble winZ; //The coordinates to pass in

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	GLdouble objx = world.m_x;
	GLdouble objy = world.m_y;
	GLdouble objz = world.m_z;

	// use the real GL view port
	glGetIntegerv(GL_VIEWPORT, viewport); 
	gluProject (objx, objy, objz, modelview, projection, viewport, &winX, &winY, &winZ);

	winY = (dFloat)viewport[3] - winY; 

	return dVector (dFloat(winX), dFloat (winY), dFloat(winZ), 0.0f);
}

*/

dVector ScreenToWorld (const dVector& screen)
{
	//Where the values will be stored
	GLint viewport[4]; 

	//Retrieves the viewport and stores it in the variable
	glGetIntegerv(GL_VIEWPORT, viewport); 

	//Where the 16 doubles of the matrix are to be stored
	GLdouble modelview[16]; 

	//Retrieve the matrix
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 

	GLdouble projection[16]; 
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	GLdouble winX = 0.0;
	GLdouble winY = 0.0;
	GLdouble winZ = 0.0; //The coordinates to pass in

	winX = screen.m_x; //Store the x cord
	winY = screen.m_y; //Store the y cord
	winZ = screen.m_z; //Store the Z cord

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	winY = (dFloat)viewport[3] - winY; 

	GLdouble objx;
	GLdouble objy;
	GLdouble objz;

	// use the real GL view port
	glGetIntegerv(GL_VIEWPORT, viewport); 
	gluUnProject (winX, winY, winZ, modelview, projection, viewport, &objx, &objy, &objz);

	return dVector (dFloat(objx), dFloat(objy), dFloat(objz));
}




int InitGraphicsSystem (int width, int height)
{
	int bpp = 0;
	int flags = 0;
	float aspectRatio; 
	const SDL_VideoInfo* info;

	// First, initialize SDL's video subsystem. 
	if( SDL_Init( SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0 ) {
		return 0;
	}


    // Let's get some video information. 
    info = SDL_GetVideoInfo( );
    if( !info ) {
		SDL_Quit( );
		return 0;
    }

    // Set our width/height to 640/480 (you would
    bpp = info->vfmt->BitsPerPixel;
    
	//  Now, we want to setup our requested window attributes for our OpenGL window.
    SDL_GL_SetAttribute (SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute (SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute (SDL_GL_BLUE_SIZE, 8 );
    SDL_GL_SetAttribute (SDL_GL_DEPTH_SIZE, 24 );
    SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, 1 );
	SDL_GL_SetAttribute (SDL_GL_SWAP_CONTROL, 0 );
	

    //flags = SDL_OPENGL | SDL_FULLSCREEN;
	flags = SDL_OPENGL;


    // Set the video mode
    if (SDL_SetVideoMode( width, height, bpp, flags )== 0) {
		SDL_Quit( );
		return 0;
    }
    // At this point, we should have a properly setup, double-buffered window for use with OpenGL.

	char titleName[2048];
	GetModuleFileNameA(NULL, titleName, 2048);
	*strstr (titleName, ".exe") = 0;
	SDL_WM_SetCaption (strrchr (titleName, '\\')  + 1, NULL);


	// Init the Font System 
	InitFont ();

    // Our shading model--Gouraud (smooth). 
    glShadeModel( GL_SMOOTH );

    // Culling. 
    glCullFace( GL_BACK );
    glFrontFace( GL_CCW );
    glEnable( GL_CULL_FACE );

    // Set the clear color. 
	 glClearColor (0.5f, 0.5f, 0.5f, 0);
//	 glClearColor (0.0f, 0.0f, 0.0f, 0);

    // Setup our viewport. 
    glViewport( 0, 0, width, height );
    
    //  Change to the projection matrix and set our viewing volume.
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );

	// set the projection matrix and viewport
    aspectRatio = (float) width / (float) height;
	glLoadIdentity();
    gluPerspective( 60.0, aspectRatio, 0.1f, 1024.0 );
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	// set one ambient directional light
	GLfloat lightColor[] = { 1.0f, 1.0f, 1.0f, 0.0 };
	GLfloat lightAmbientColor[] = { 0.7f, 0.7f, 0.7f, 0.0 };
	GLfloat lightPosition[] = { 500.0f, 200.0f, 500.0f, 0.0 };

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbientColor);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightColor);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	return 1;
}


dVector GetMousePos()
{
	int x;
	int y;
//	POINT point;
//	GetCursorPos(&point);

	SDL_GetMouseState(&x, &y);
	return dVector (dFloat (x), dFloat (y), 0.0f);
}


int IsKeyDown (int keycode)
{
	int code;
	code = GetAsyncKeyState (keycode);

if (code & 0x8000){
	code *= 1;
}

	return (code & 0x8000) ? 1 : 0;
}


void ProcessEvents (NewtonWorld* world)
{
	// Our SDL event placeholder. 
	SDL_Event event;

	// Grab all the events off the queue. 
	while( SDL_PollEvent( &event ) ) {
		switch (event.type) 
		{
			case SDL_KEYDOWN:
			{
				switch ( event.key.keysym.sym ) 
				{
					case SDLK_ESCAPE:
					{
						// Handle quit requests. 
						SDL_Quit( );
						exit (0);
						break;
					}

					default: 
					{
						break;
					}
				}
				break;
			}

			case SDL_MOUSEMOTION:
			case SDL_MOUSEBUTTONDOWN:
			{
				break;
			}

			case SDL_QUIT:
			{
				// Handle quit requests. 
				SDL_Quit( );

				// Exit program. 
				exit (0);
				break;
			}
		}
	}

	InputControl(world);
}




// this function set a Open GL Camera
void SetCamera (const dVector& eyePoint, const dVector& target)
{
	glMatrixMode(GL_MODELVIEW);

//	glLoadIdentity();
//	gluLookAt(eye.m_x, eye.m_y, eye.m_z, target.m_x, target.m_y, target.m_z, 0.0, 1.0, 0.0);

	// compatibility with GL ES, build the camera matrix by hand for 
	dMatrix camera (GetIdentityMatrix());
	dVector front (eyePoint - target);

	camera.m_right = front.Scale (1.0f / dSqrt (front % front));
	camera.m_front = camera.m_up * camera.m_right;
	camera.m_front = camera.m_front.Scale (1.0f / dSqrt (camera.m_front % camera.m_front));
	camera.m_up    = camera.m_right * camera.m_front;
	camera = camera.Transpose();
	camera.m_posit -= camera.RotateVector (eyePoint);
	camera.m_posit.m_w = 1.0f;

	glLoadIdentity();
	glMultMatrixf(&camera[0][0]);
}





