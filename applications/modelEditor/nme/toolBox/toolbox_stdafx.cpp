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
