/////////////////////////////////////////////////////////////////////////////
// Name:        dPluginStdafx.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////



#include "dPluginStdafx.h"
#include "dPluginUtils.h"


#ifdef _DPLUGIN_COMMON_BUILD_DLL
	#if (defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
	int main(int argc, char* argv[])
	{
		return 0;
	}
	#endif

	#ifdef _MSC_VER
	BOOL APIENTRY DllMain (HANDLE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved)
	{
		return TRUE;
	}
	#endif
#endif





void GetAplicationDirectory (char* const aplicationDir)
{
#ifdef _MSC_VER
	GetModuleFileNameA(NULL, aplicationDir, 256);
#endif


#ifdef _MACOSX_VER
	//strcpy(outPathName, "/Users/admin_user/Developer/NGD/svn (trunk)/applications/media/");
	//strcat(outPathName, name);

	CFURLRef appURL = CFBundleCopyBundleURL( CFBundleGetMainBundle() ); 
	CFURLRef appDirURL = CFURLCreateCopyDeletingLastPathComponent( NULL, appURL); 
	CFStringRef fileName = CFStringCreateWithCString( NULL, name, kCFStringEncodingUTF8 ); 
	CFURLRef fileURL = CFURLCreateCopyAppendingPathComponent( NULL, appDirURL, fileName, false ); 
	CFStringRef filePath = CFURLCopyFileSystemPath( fileURL, kCFURLPOSIXPathStyle ); 
	CFStringGetCString (filePath, outPathName, PATH_MAX, kCFStringEncodingUTF8); 

	// Clean up 
	CFRelease( appURL ); 
	CFRelease( appDirURL ); 
	CFRelease( fileName ); 
	CFRelease( fileURL ); 
	CFRelease( filePath );

#endif

#ifdef _POSIX_VER
	char id[2048];
	char path[2048];

	sprintf(id, "/proc/%d/exe", getpid());
	memset (path, 0, sizeof (path));
	readlink(id, path, 1024);
#endif

	strlwr (aplicationDir);
	char* ptr = strrchr (aplicationDir, '\\');
	if (ptr) {
		ptr[1] = 0;
	} else {
		ptr = strrchr (aplicationDir, '/');
		if (ptr) {
			ptr[1] = 0;
		}
	}
	_ASSERTE (ptr);
}



void GetMediaDirectory (char* const mediaDir)
{
	char appPath [2048];
	GetAplicationDirectory (appPath);

	char* const ptr = strstr (appPath, "applications");
	if (ptr) {
		ptr[0] = 0;
		sprintf (mediaDir, "%sapplications/media", appPath);
	} else {
		*mediaDir = 0;
	}
}

// Windows user assets path
void GetWorkingFileName (const char* const name, char* const outPathName)
{
	char appPath [2048];
	GetAplicationDirectory (appPath);

	char* const ptr = strstr (appPath, "applications");
	ptr [0] = 0;
	sprintf (outPathName, "%sapplications/media/%s", appPath, name);
}




void* operator new (size_t size) 
{ 
	return NewtonAlloc(int (size));
}

void operator delete (void* ptr) 
{ 
	NewtonFree(ptr);
}


void* dPluginAlloc::operator new (size_t size)
{
	return NewtonAlloc(int (size));
}

void dPluginAlloc::operator delete (void* ptr)
{
	NewtonFree(ptr);
}

