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



#include "dCILstdafx.h"
#include "dCILInstr.h"
#include "dTransformation.h"
#include "dCILInstrMiscellaneous.h"


dTransformation::dTransformation (dCIL* const cil, dCILInstrFunction* const function)
	:m_cil (cil)
	,m_function (function)
{
	dAssert (cil);
	dAssert (function);
	dAssert (function->GetAsFunction());
}

dTransformation::~dTransformation()
{
}

