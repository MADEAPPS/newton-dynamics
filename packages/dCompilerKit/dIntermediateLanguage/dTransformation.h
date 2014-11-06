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

#ifndef _dTransformation_H_
#define _dTransformation_H_

#include "dCILstdafx.h"

class dCIL;
class dCILInstrFunction;

class dTransformation
{
	public:
	dTransformation (dCIL* const cil, dCILInstrFunction* const function);
	virtual ~dTransformation();
	virtual bool Update() = 0;

	dCIL* m_cil;
	dCILInstrFunction* const m_function;
};

#endif