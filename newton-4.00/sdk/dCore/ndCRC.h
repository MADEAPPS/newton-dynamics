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

#ifndef __NDCRC_H__
#define __NDCRC_H__

#include "ndCoreStdafx.h"

D_CORE_API ndUnsigned64 dCRC64 (const char* const string, ndUnsigned64  crcAcc = 0);
D_CORE_API ndUnsigned64 dCRC64 (const void* const buffer, ndInt32 size, ndUnsigned64 crcAcc);

inline ndUnsigned64 dCombineCRC(ndUnsigned64 a, ndUnsigned64 b)
{
	return (a << 8) ^ b;
}


#endif

