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

#if (defined(WIN32) || defined(_WIN32))
#include <windows.h>
#include <commdlg.h>
#endif

#include "ndFileBrowser.h"

bool dGetOpenFileNamePLY(char* const fileName, int maxSize)
{
#if (defined(WIN32) || defined(_WIN32))
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(nullptr, appPath, sizeof (appPath));
	_strlwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\media");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof (ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = maxSize;
	ofn.lpstrFilter = const_cast<LPSTR>("import file *.ply\0*.ply\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton Dynamics demos");
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
#if (defined(WIN32) || defined(_WIN32))
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(nullptr, appPath, sizeof (appPath));
	_strlwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\media");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof (ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = maxSize;
	ofn.lpstrFilter = const_cast<LPSTR>("newton serialized file *.bin\0*.bin\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton Dynamics demos");
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetOpenFileName(&ofn) ? true : false;
	return state;
#else
	return false;
#endif
}

bool dGetSaveFileNameSerialization(char* const fileName, int maxSize)
{
#if (defined(WIN32) || defined(_WIN32))
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
	_strlwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\media");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = maxSize;
	ofn.lpstrFilter = const_cast<LPSTR>("newton serialized file *.bin\0*.bin\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton Dynamics demos");
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

bool dGetSaveNdFileName(char* const fileName, int maxSize)
{
#if (defined(WIN32) || defined(_WIN32))
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
	_strlwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\ndSandbox");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = maxSize;
	ofn.lpstrFilter = const_cast<LPSTR>("newton save file *.nd\0*.nd\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton Dynamics 4.0 demos");
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetSaveFileName(&ofn) ? true : false;
	if (state) 
	{
		char* const ext = strrchr(fileName, '.');
		if (!ext) 
		{
			strcat(fileName, ".nd");
		}
	}
	return state;
#else
	return false;
#endif
}

bool dGetLoadNdFileName(char* const fileName, int maxSize)
{
#if (defined(WIN32) || defined(_WIN32))
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
	_strlwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\ndSandbox");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = maxSize;
	ofn.lpstrFilter = const_cast<LPSTR>("newton load file *.nd\0*.nd\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton Dynamics 4.0 demos");
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetOpenFileName(&ofn) ? true : false;
	if (state)
	{
		char* const ext = strrchr(fileName, '.');
		if (!ext)
		{
			strcat(fileName, ".nd");
		}
	}
	return state;
#else
	return false;
#endif
}
