/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// stdafx.cpp : source file that includes just the standard includes
// NewView.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information

#include "ndSandboxStdafx.h"

// Windows user assets path
void ndGetWorkingFileName (const char* const name, char* const outPathName)
{
	#if (defined(WIN32) || defined(_WIN32))
		char appPath [256];
		GetModuleFileNameA(nullptr, appPath, sizeof (appPath));
		strtolwr(appPath);

		char* const end = strstr (appPath, "applications");
		end [0] = 0;
		snprintf (outPathName, sizeof (appPath), "%sapplications/media/%s", appPath, name);
	#elif defined(__APPLE__)
        char tmp[2048];
		CFURLRef appURL (CFBundleCopyBundleURL(CFBundleGetMainBundle()));
        CFStringRef filePath (CFURLCopyFileSystemPath (appURL, kCFURLPOSIXPathStyle));
        CFStringGetCString (filePath, tmp, PATH_MAX, kCFStringEncodingUTF8);
        //char* const ptr = strstr (tmp, "applications");
        //ptr [0] = 0;
        snprintf (outPathName, sizeof (tmp) + 20, "%s/Contents/Resources/%s", tmp, name);

		// Clean up 
		CFRelease( appURL ); 
		CFRelease( filePath );
	#elif defined(__linux__)
		char id[2048];
		char appPath[2048];

		snprintf(id, sizeof (id), "/proc/%d/exe", getpid());
		memset (appPath, 0, sizeof (appPath));
		size_t ret = readlink(id, appPath, sizeof (appPath));
		ret = 0;
		char* const end = strstr (appPath, "applications");
		*end = 0;
		snprintf (outPathName, sizeof(appPath) + 20, "%sapplications/media/%s", appPath, name);
	#else
		#error  "error: need to implement \"dGetWorkingFileName\" here for this platform"
	#endif
}

ndString ndGetWorkingFileName(const char* const name)
{
	char outPathName[1024];
	ndGetWorkingFileName(name, outPathName);
	return ndString(outPathName);
}

ndUnsigned16 ndIndian16(ndUnsigned16 x)
{
	return ndUnsigned16 (((x >> 8) & 0xff) + ((x & 0xff) << 8));
}

ndUnsigned32 ndIndian32(ndUnsigned32 x)
{
	ndUnsigned16 low = ndUnsigned16(x);
	ndUnsigned16 high = ndUnsigned16(x>>16);
	return ndUnsigned32(ndIndian16(high)) + (ndUnsigned32(ndIndian16(low)) << 16);
}

// endian conversion
#ifdef __ppc__
	ndUnsigned16 SWAP_INT16(ndUnsigned16 x)
	{
		return ((x >> 8) & 0xff) + ((x & 0xff) << 8);
	}

	ndUnsigned32 SWAP_INT32(ndUnsigned32 x)
	{
		return SWAP_INT16 ( x >> 16) + (SWAP_INT16 (x) << 16);
	}

//	void SWAP_FLOAT32_ARRAY (void* const array, ndInt32 count)
//	{
//		ndInt32* const ptr = (ndInt32*) array;
//		count /= sizeof (ndInt32);
//		for (ndInt32 i = 0; i < count; ++i) 
//		{
//			ndInt32 x;
//			x = SWAP_INT32 (ptr[i]);
//			ptr[i] = x;
//		}
//	}

#else

	ndUnsigned32 SWAP_INT32(ndUnsigned32 x)
	{
		return x;
	}

	ndUnsigned16 SWAP_INT16(ndUnsigned16 x)
	{
		return x;
	}

//	void SWAP_FLOAT32_ARRAY (void* const, ndInt32)
//	{
//	}
#endif
