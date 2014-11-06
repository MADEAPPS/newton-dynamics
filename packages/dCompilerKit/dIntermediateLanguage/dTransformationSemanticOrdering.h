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

#ifndef _dTransformationSemanticOrdering_H_
#define _dTransformationSemanticOrdering_H_

#include "dCILstdafx.h"
#include "dTransformation.h"


class dTransformationSemanticOrdering: public dTransformation
{
	public:
	dTransformationSemanticOrdering (dCIL* const cil, dCILInstrFunction* const function);
	virtual ~dTransformationSemanticOrdering();
	virtual bool Update();
};

#endif