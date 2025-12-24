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

#include "newtonStdafx.h"
#include "newtonConfig.h"

void *operator new (size_t size)
{
	// this should not happens on this test
	// newton should never use global operator new and delete.
	
	static bool allowStandardThreadAllocation = true;
	void* const ptr = ndMemory::Malloc(size);
	ndAssert(allowStandardThreadAllocation || ((ndUnsigned64(ptr) & (0x1f)) == 0));
	allowStandardThreadAllocation = false;
	return ptr;
}

void operator delete (void* ptr) noexcept
{
	//dAssert(0);
	ndMemory::Free(ptr);
}

// Windows user assets path
void dGetWorkingFileName (const char* const name, char* const outPathName)
{
	#if (defined(WIN32) || defined(_WIN32))
		char appPath [256];
		GetModuleFileNameA(nullptr, appPath, sizeof (appPath));
		_strlwr (appPath);

		char* const end = strstr (appPath, "applications");
		end [0] = 0;
		sprintf (outPathName, "%sapplications/media/%s", appPath, name);
	#elif defined(__APPLE__)
        char tmp[2048];
		CFURLRef appURL (CFBundleCopyBundleURL(CFBundleGetMainBundle()));
        CFStringRef filePath (CFURLCopyFileSystemPath (appURL, kCFURLPOSIXPathStyle));
        CFStringGetCString (filePath, tmp, PATH_MAX, kCFStringEncodingUTF8);
        //char* const ptr = strstr (tmp, "applications");
        //ptr [0] = 0;
        //sprintf (outPathName, "%sapplications/media/%s", tmp, name);
        sprintf (outPathName, "%s/Contents/Resources/%s", tmp, name);

		// Clean up 
		CFRelease( appURL ); 
		CFRelease( filePath );
	#elif defined(__linux__)
		char id[2048];
		char appPath[2048];

		sprintf(id, "/proc/%d/exe", getpid());
		memset (appPath, 0, sizeof (appPath));
		size_t ret = readlink(id, appPath, sizeof (appPath));
		ret = 0;
		char* const end = strstr (appPath, "applications");
		*end = 0;
		sprintf (outPathName, "%sapplications/media/%s", appPath, name);
	#else
		#error  "error: need to implement \"dGetWorkingFileName\" here for this platform"
	#endif
}


