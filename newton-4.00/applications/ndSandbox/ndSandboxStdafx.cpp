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

static dUnsigned32 ___dRandSeed___ = 0;

void dSetRandSeed (dUnsigned32 seed)
{
	___dRandSeed___	= seed; 
}

dInt32 dRandInt()
{
	// numerical recipe in c
	#define RAND_MUL 1664525u
	#define RAND_ADD 1013904223u
	___dRandSeed___ = RAND_MUL * ___dRandSeed___ + RAND_ADD;
	return ___dRandSeed___ & dRAND_MAX;
}


/// return a random variable between 0.0 and 1.0
dFloat32 dRand()
{
	// numerical recipe in c
	dFloat32 r = dFloat32(dRandInt()) * ((dFloat32(1.0f) / dRAND_MAX));
	//dTrace(("%f\n", r));
	return r;
}

/// return a pseudo Gaussian random with mean 0 and variance 0.5f
dFloat32 dGaussianRandom (dFloat32 amp)
{
	const dInt32 count = 4;
	dFloat32 r = dFloat32(0.0f);
	for (dInt32 i = 0; i < count; i++)
	{
		r += dFloat32(2.0f) * dRand() - dFloat32 (1.0f);
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
	dUnsigned16 SWAP_INT16(dUnsigned16 short x)
	{
		return ((x >> 8) & 0xff) + ((x & 0xff) << 8);
	}

	dUnsigned32 SWAP_INT32(dUnsigned32 x)
	{
		return SWAP_INT16 ( x >> 16) + (SWAP_INT16 (x) << 16);
	}

	void SWAP_FLOAT32_ARRAY (void* const array, dInt32 count)
	{
		dInt32* const ptr = (dInt32*) array;
		count /= sizeof (dInt32);
		for (dInt32 i = 0; i < count; i ++) 
		{
			dInt32 x;
			x = SWAP_INT32 (ptr[i]);
			ptr[i] = x;
		}
	}

#else

	dUnsigned32 SWAP_INT32(dUnsigned32 x)
	{
		return x;
	}

	dUnsigned16 SWAP_INT16(dUnsigned16 x)
	{
		return x;
	}

	void SWAP_FLOAT32_ARRAY (void* const, dInt32)
	{
	}
#endif
