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


// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include <windows.h>
#include <commdlg.h>

#include "FileBrowser.h"

bool dGetOpenFileName (char* const fileName, int maxSize)
{
#ifdef _WIN32 
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(NULL, appPath, sizeof (appPath));
	_strlwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat (appPath, "applications\\media");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof (ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = maxSize;
	ofn.lpstrFilter = "newton files\0*.ngd\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = "Newton Dynamics demos";
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	return GetOpenFileName(&ofn) ? true : false;
#else
	return false;
#endif
}

bool dGetSaveFileName(char* const fileName, int maxSize)
{
#ifdef _WIN32 
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(NULL, appPath, sizeof(appPath));
	_strlwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\media");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = maxSize;
	ofn.lpstrFilter = "newton files\0*.ngd\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = "Newton Dynamics demos";
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	return GetSaveFileName(&ofn) ? true : false;
#else
	return false;
#endif
}
