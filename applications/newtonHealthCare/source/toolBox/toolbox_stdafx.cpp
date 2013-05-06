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

// stdafx.cpp : source file that includes just the standard includes
// NewView.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information


#include <toolbox_stdafx.h>

// TODO: reference any additional headers you need in STDAFX.H
// and not in this file

unsigned dRand ()
{
	#define RAND_MUL 31415821u
	static unsigned randSeed = 0;
	randSeed = RAND_MUL * randSeed + 1; 
	return randSeed & dRAND_MAX;
}



// Windows user assets path
void GetWorkingFileName (const char* const name, char* const outPathName)
{
#ifdef _MSC_VER
	char path[256]; 
	GetModuleFileNameA(NULL, path, 256);

	strlwr (path);
	char* ptr = strstr (path, "applications");
	ptr [0] = 0;
	sprintf (outPathName, "%sapplications/media/%s", path, name);
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
	
	char* const ptr = strstr (outPathName, "applications");
	sprintf (ptr, "applications/media/%s", name);

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
	char* const end = strstr (path, "/applications");
	*end = 0;
	sprintf (outPathName, "%s/applications/media/%s", path, name);
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
