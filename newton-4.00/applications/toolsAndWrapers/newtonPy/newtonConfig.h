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


#ifndef _NEWTON_PY_CONFIG_H_
#define _NEWTON_PY_CONFIG_H_


#ifdef D_CLASS_REFLECTION
	#undef D_CLASS_REFLECTION
#endif

#ifdef D_MSV_NEWTON_ALIGN_16
	#undef D_MSV_NEWTON_ALIGN_16
#endif

#ifdef D_MSV_NEWTON_ALIGN_32
	#undef D_MSV_NEWTON_ALIGN_32
#endif

#ifdef D_CORE_API 
	#undef D_CORE_API 
#endif

#ifdef D_COLLISION_API
	#undef D_COLLISION_API
#endif

#ifdef D_NEWTON_API
	#undef D_NEWTON_API
#endif

#ifdef D_OPERATOR_NEW_AND_DELETE
	#undef D_OPERATOR_NEW_AND_DELETE
#endif

#define D_CORE_API 
#define D_NEWTON_API
#define D_COLLISION_API
#define D_OPERATOR_NEW_AND_DELETE

//#define D_BASE_CLASS_REFLECTION(x)
#define D_CLASS_REFLECTION(x,y)
#define D_MSV_NEWTON_ALIGN_16
#define D_GCC_NEWTON_ALIGN_16
#define D_MSV_NEWTON_ALIGN_32
#define D_GCC_NEWTON_ALIGN_32

//#define ndFloat32 float
//#define ndFloat64 double
//#define ndInt8 signed char
//#define ndInt16 signed short
//#define ndInt32 signed int
//#define ndInt64 signed long long
//#define ndUnsigned8 unsigned char
//#define ndUnsigned16 unsigned short
//#define ndUnsigned32 unsigned int
//#define ndUnsigned64 unsigned long long

//class XXXXX
//{
//	public:
//	int xxxxxxxx0;
//	dInt32 xxxxxxxx1;
//
//	int Get0() {return xxxxxxxx0;}
//	dInt32 Get1() { return xxxxxxxx0; }
//};

void dGetWorkingFileName(const char* const name, char* const outPathName);
#endif 

