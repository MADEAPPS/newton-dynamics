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

#ifndef __DRTTI_H__
#define __DRTTI_H__

#include "dCRC.h"

#define dRttiCommon(className)				\
	private:								\
	static dCRCTYPE m_rtti; 				\
	public:									\
	public:									\
	static dCRCTYPE GetRttiType()			\
	{										\
		return m_rtti;						\
	}										\
	virtual dCRCTYPE GetTypeId () const		\
	{										\
		return m_rtti;						\
	}										\



// add these macros only to the root base class that you want to have rtti 
#define dRttiRootClassSupportDeclare(className)		\
	dRttiCommon(className)							\
	virtual bool IsType (dCRCTYPE typeId) const		\
	{												\
		return typeId == m_rtti;					\
	}												

#define dRttiRootClassSupportImplement(className)	\
	dCRCTYPE className::m_rtti = dCRC64 (#className);



// add these macros to every derived class  
#define dAddRtti(baseClass)								\
	dRttiCommon(baseClass)								\
	virtual bool IsType (dCRCTYPE typeId) const			\
	{													\
		if (typeId == m_rtti) {							\
			return true;								\
		}												\
		return baseClass::IsType (typeId);				\
	}													


#define dInitRtti(className)							\
	dRttiRootClassSupportImplement(className)


#endif