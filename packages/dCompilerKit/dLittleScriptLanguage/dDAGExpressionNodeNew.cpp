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

#include "dLSCstdafx.h"
#include "dDAG.h"
#include "dDAGTypeNode.h"
#include "dDAGDimensionNode.h"
#include "dDAGScopeBlockNode.h"
#include "dDAGExpressionNodeNew.h"


dInitRtti(dDAGExpressionNodeNew);

dDAGExpressionNodeNew::dDAGExpressionNodeNew(dList<dDAG*>& allNodes, const char* const identifierType, dDAGDimensionNode* const dimension)
	:dDAGExpressionNode(allNodes)
	,m_dimension (dimension)
{
	m_name = identifierType;
}


dDAGExpressionNodeNew::~dDAGExpressionNodeNew(void)
{
}

void dDAGExpressionNodeNew::ConnectParent(dDAG* const parent)  
{
dAssert (0);
	m_parent = parent;
	m_dimension->ConnectParent(this)  ;
}


void dDAGExpressionNodeNew::CompileCIL(dCIL& cil)
{
	dAssert (0);
	/*

	m_dimension->CompileCIL(cil);

	dTreeAdressStmt& arraySizeInBytes = cil.NewStatement()->GetInfo();
	arraySizeInBytes.m_instruction = dTreeAdressStmt::m_assigment;
	arraySizeInBytes.m_operator = dTreeAdressStmt::m_mul;
	arraySizeInBytes.m_arg0.m_label = cil.NewTemp();
	arraySizeInBytes.m_arg1 = m_dimension->m_result;

	arraySizeInBytes.m_arg2.m_type = dTreeAdressStmt::m_intConst;
	arraySizeInBytes.m_arg2.m_label = "1";
	if (m_name == "int") {
		arraySizeInBytes.m_arg2.m_label = "4"; 
	} else {
		dAssert (0);
	}
	DTRACE_INTRUCTION (&arraySizeInBytes);

	dTreeAdressStmt& operatorNewInstruction = cil.NewStatement()->GetInfo();
	operatorNewInstruction.m_instruction = dTreeAdressStmt::m_new;
	operatorNewInstruction.m_arg0.m_label = cil.NewTemp();
	operatorNewInstruction.m_arg1.m_label = arraySizeInBytes.m_arg0.m_label; 
	DTRACE_INTRUCTION (&operatorNewInstruction);

	m_result = operatorNewInstruction.m_arg0;

	dDAGScopeBlockNode* const scope = GetScope();
	scope->m_allocations.Append(m_result.m_label);
*/
}
