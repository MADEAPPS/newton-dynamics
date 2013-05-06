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

#ifndef __dCRC__
#define __dCRC__

#define dCRCTYPE long long

dCRCTYPE dCRC64 (const char* const string, dCRCTYPE  crcAcc = 0);
dCRCTYPE dCRC64 (const void* const buffer, int size, dCRCTYPE crcAcc);

dCRCTYPE dCombineCRC (dCRCTYPE a, dCRCTYPE b);

#endif

