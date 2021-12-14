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

// stdafx.cpp : source file that includes just the standard includes
// NewView.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information


#include "ndSandboxStdafx.h"

// TODO: reference any additional headers you need in STDAFX.H
// and not in this file

static ndUnsigned32 ___dRandSeed___ = 0;

void dSetRandSeed (ndUnsigned32 seed)
{
	___dRandSeed___	= seed; 
}

ndInt32 dRandInt()
{
	// numerical recipe in c
	#define RAND_MUL 1664525u
	#define RAND_ADD 1013904223u
	___dRandSeed___ = RAND_MUL * ___dRandSeed___ + RAND_ADD;
	return ___dRandSeed___ & dRAND_MAX;
}


/// return a random variable between 0.0 and 1.0
ndFloat32 dRand()
{
	// numerical recipe in c
	ndFloat32 r = ndFloat32(dRandInt()) * ((ndFloat32(1.0f) / dRAND_MAX));
	//dTrace(("%f\n", r));
	return r;
}

/// return a pseudo Gaussian random with mean 0 and variance 0.5f
ndFloat32 dGaussianRandom (ndFloat32 amp)
{
	const ndInt32 count = 4;
	ndFloat32 r = ndFloat32(0.0f);
	for (ndInt32 i = 0; i < count; i++)
	{
		r += ndFloat32(2.0f) * dRand() - ndFloat32 (1.0f);
	}
	r *= (amp / count);
	//dTrace(("%f\n", r));
	return r;
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

	// endian conversion
#ifdef __ppc__
	ndUnsigned16 SWAP_INT16(ndUnsigned16 short x)
	{
		return ((x >> 8) & 0xff) + ((x & 0xff) << 8);
	}

	ndUnsigned32 SWAP_INT32(ndUnsigned32 x)
	{
		return SWAP_INT16 ( x >> 16) + (SWAP_INT16 (x) << 16);
	}

	void SWAP_FLOAT32_ARRAY (void* const array, ndInt32 count)
	{
		ndInt32* const ptr = (ndInt32*) array;
		count /= sizeof (ndInt32);
		for (ndInt32 i = 0; i < count; i ++) 
		{
			ndInt32 x;
			x = SWAP_INT32 (ptr[i]);
			ptr[i] = x;
		}
	}

#else

	ndUnsigned32 SWAP_INT32(ndUnsigned32 x)
	{
		return x;
	}

	ndUnsigned16 SWAP_INT16(ndUnsigned16 x)
	{
		return x;
	}

	void SWAP_FLOAT32_ARRAY (void* const, ndInt32)
	{
	}
#endif
