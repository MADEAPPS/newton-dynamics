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

// stdafx.cpp : source file that includes just the standard includes
// NewView.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information


#include "toolbox_stdafx.h"

// TODO: reference any additional headers you need in STDAFX.H
// and not in this file

void *operator new (size_t size) 
{ 
	void* const ptr = ::malloc (size);
		//unsigned xxx = unsigned (ptr);
		//xxx &= 0xffff;
		////dAssert (xxx != 0x2378);
		//dAssert (!((xxx == 0x2378) && (size == 12)));
		//dTrace (("%d %x\n", xxx, ptr))
	return ptr; 
}                                          

void operator delete (void* ptr) 
{ 
	::free (ptr); 
}


static unsigned ___dRandSeed___ = 0;

void dSetRandSeed (unsigned seed)
{
	___dRandSeed___	= seed; 
}

unsigned dRand ()
{
	#define RAND_MUL 31415821u
	___dRandSeed___ = RAND_MUL * ___dRandSeed___ + 1; 
	return ___dRandSeed___ & dRAND_MAX;
}

// a pseudo Gaussian random with mean 0 and variance 0.5f
dFloat dGaussianRandom (dFloat amp)
{
	unsigned val;
	val = dRand() + dRand();
	return amp * (dFloat (val) / dFloat(dRAND_MAX) - 1.0f) * 0.5f;
}


// Windows user assets path
void dGetWorkingFileName (const char* const name, char* const outPathName)
{
	#if defined (_MSC_VER)

		//GetAplicationDirectory (appPath);
		char appPath [256];
		GetModuleFileNameA(NULL, appPath, sizeof (appPath));
		strlwr (appPath);

		char* const end = strstr (appPath, "applications");
		end [0] = 0;
		sprintf (outPathName, "%sapplications/media/%s", appPath, name);

	#elif defined(_MACOSX_VER)
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
	
	#elif (defined (_POSIX_VER) || defined (_POSIX_VER_64))

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

	// little Indian/big Indian conversion
#ifdef __ppc__
	unsigned short SWAP_INT16(unsigned short x)
	{
		return ((x >> 8) & 0xff) + ((x & 0xff) << 8);
	}
	unsigned SWAP_INT32(unsigned x)
	{
		return SWAP_INT16 ( x >> 16) + (SWAP_INT16 (x) << 16);
	}


	void SWAP_FLOAT32_ARRAY (void* const array, dInt32 count)
	{
		dInt32* const ptr = (dInt32*) array;
		count /= sizeof (dInt32);
		for (dInt32 i = 0; i < count; i ++) {
			dInt32 x;
			x = SWAP_INT32 (ptr[i]);
			ptr[i] = x;
		}
	}

#else

	unsigned SWAP_INT32(unsigned x)
	{
		return x;
	}

	unsigned short SWAP_INT16(unsigned short x)
	{
		return x;
	}

	void SWAP_FLOAT32_ARRAY (void* const array, dInt32 count)
	{
	}
#endif
