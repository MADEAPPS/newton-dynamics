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
#include "dCIL.h"
#include "dCILInstr.h"
#include "dCILInstrMiscellaneous.h"
#include "dTransformationSemanticOrdering.h"


dTransformationSemanticOrdering::dTransformationSemanticOrdering (dCIL* const cil, dCILInstrFunction* const function)
	:dTransformation (cil, function)
{
}

dTransformationSemanticOrdering::~dTransformationSemanticOrdering()
{
}

bool dTransformationSemanticOrdering::Update()
{
	bool ret = false;
	for (dCIL::dListNode* stmtNode = m_function->GetNode(); !stmtNode->GetInfo()->GetAsFunctionEnd(); stmtNode = stmtNode->GetNext()) {
		ret |= stmtNode->GetInfo()->ApplySemanticReordering();
	}
	return ret;
}

