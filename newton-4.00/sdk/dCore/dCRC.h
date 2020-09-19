/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __dCRC_H__
#define __dCRC_H__

#include "dCoreStdafx.h"

D_CORE_API dUnsigned64 dCRC64 (const char* const string, dUnsigned64  crcAcc = 0);
D_CORE_API dUnsigned64 dCRC64 (const void* const buffer, dInt32 size, dUnsigned64 crcAcc);

inline dUnsigned64 dCombineCRC(dUnsigned64 a, dUnsigned64 b)
{
	return (a << 8) ^ b;
}


#endif

