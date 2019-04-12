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
#ifdef _WIN32 
#include <windows.h>
#include <commdlg.h>
#endif

#include "FileBrowser.h"

bool dGetOpenFileNameNgd (char* const fileName, int maxSize)
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
	ofn.lpstrFilter = "newton xml files *.ngd\0*.ngd\0";
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

bool dGetOpenFileNameSerialization(char* const fileName, int maxSize)
{
#ifdef _WIN32 
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(NULL, appPath, sizeof (appPath));
	_strlwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\media");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof (ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = maxSize;
	ofn.lpstrFilter = "newton serialized file *.bin\0*.bin\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = "Newton Dynamics demos";
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetOpenFileName(&ofn) ? true : false;
	return state;
#else
	return false;
#endif
}

bool dGetSaveFileNameNgd(char* const fileName, int maxSize)
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
	ofn.lpstrFilter = "newton xml files *.ngd\0*.ngd\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = "Newton Dynamics demos";
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetSaveFileName(&ofn) ? true : false;
	if (state) {
		char* const ext = strrchr(fileName, '.');
		if (!ext) {
			strcat(fileName, ".ngd");
		}
	}
	return state;

#else
	return false;
#endif
}

bool dGetSaveFileNameSerialization(char* const fileName, int maxSize)
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
	ofn.lpstrFilter = "newton serialized file *.bin\0*.bin\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = "Newton Dynamics demos";
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetSaveFileName(&ofn) ? true : false;
	if (state) {
		char* const ext = strrchr (fileName, '.');
		if (!ext) {
			strcat (fileName, ".bin");
		}
	}
	return state;
#else
	return false;
#endif

}
