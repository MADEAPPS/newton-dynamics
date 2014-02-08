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
#include "dDAGExpressionNodeVariable.h"
#include "dDAGExpressionNodeAssigment.h"

dInitRtti(dDAGExpressionNodeVariable);

dDAGExpressionNodeVariable::dDAGExpressionNodeVariable(dList<dDAG*>& allNodes, const char* const identifier, dDAGDimensionNode* const expressionDimIndex)
	:dDAGExpressionNode(allNodes)
	,m_type(NULL)
	,m_dimExpressions ()
{
	m_name = identifier;

	dDAGDimensionNode* next;
	for (dDAGDimensionNode* node = expressionDimIndex; node; node = next) {
		next = node->m_next;
		node->m_next = NULL;
		dAssert (node->IsType(dDAGDimensionNode::GetRttiType()));
		m_dimExpressions.Append(node);
	}
}

dDAGExpressionNodeVariable::dDAGExpressionNodeVariable (dList<dDAG*>& allNodes, const dDAGExpressionNodeVariable& copySource)
	:dDAGExpressionNode(allNodes)
{
	m_name = copySource.m_name;
	for (dList<dDAGDimensionNode*>::dListNode* node = copySource.m_dimExpressions.GetFirst(); node; node = node->GetNext()) {
		m_dimExpressions.Append((dDAGDimensionNode*) node->GetInfo()->Clone (allNodes));
	}
}

dDAGExpressionNodeVariable::~dDAGExpressionNodeVariable(void)
{
}

void dDAGExpressionNodeVariable::SetType(dDAGTypeNode* const type)
{
	m_type = type;
}

dDAG* dDAGExpressionNodeVariable::Clone (dList<dDAG*>& allNodes) const
{
	return new dDAGExpressionNodeVariable (allNodes, *this);
}

void dDAGExpressionNodeVariable::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	if (m_type) {
		m_type->ConnectParent(this);
	}

	for (dList<dDAGDimensionNode*>::dListNode* node = m_dimExpressions.GetFirst(); node; node = node->GetNext()) {
		dDAGDimensionNode* const dim = node->GetInfo();
		dim->ConnectParent(this);
	}
}


void dDAGExpressionNodeVariable::CompileCIL(dCIL& cil)
{
	dString variable (m_name);
	int pos = variable.Find (D_SCOPE_PREFIX, 0, int (strlen (D_SCOPE_PREFIX)));
	if (pos != 0) {
		bool state = RenameLocalVariable(cil, variable);
		if (!state) {
			dTrace (("undefined local variable\n"));
			dAssert (0);
		}
	}

	if (m_dimExpressions.GetCount()) {
		dDAGDimensionNode* const dim = m_dimExpressions.GetFirst()->GetInfo();
		dim->CompileCIL(cil);
		dCIL::dListNode* const dimInstruction = cil.NewStatement();
		dTreeAdressStmt& addressIndex = dimInstruction->GetInfo();
		addressIndex.m_instruction = dTreeAdressStmt::m_assigment;
		addressIndex.m_arg0.m_label = cil.NewTemp();
		addressIndex.m_arg1 = dim->m_result; 

		dString result = addressIndex.m_arg0.m_label;
		DTRACE_INTRUCTION (&addressIndex);

		for (dList<dDAGDimensionNode*>::dListNode* node = m_dimExpressions.GetFirst()->GetNext(); node; node = node->GetNext()) {
			dAssert (0);
/*
			dDAGDimensionNode* const dim = node->GetInfo();
			dim->CompileCIL(cil);
			
			dTreeAdressStmt& stmtMul = cil.NewStatement()->GetInfo();
			stmtMul.m_instruction = dTreeAdressStmt::m_assigment;
			stmtMul.m_operator = dTreeAdressStmt::m_mul;
			stmtMul.m_arg0.m_label = cil.NewTemp();
			stmtMul.m_arg1.m_label = result;
			stmtMul.m_arg2.m_label = dim->m_arraySize;

			DTRACE_INTRUCTION (&stmtMul);

			dTreeAdressStmt& stmtAdd = cil.NewStatement()->GetInfo();
			stmtAdd.m_instruction = dTreeAdressStmt::m_assigment;
			stmtAdd.m_operator = dTreeAdressStmt::m_add;
			stmtAdd.m_arg0.m_label = cil.NewTemp();
			stmtAdd.m_arg1.m_label = stmtMul.m_arg0.m_label;
			stmtAdd.m_arg2 = dim->m_result;

			result = stmtAdd.m_arg0.m_label;

			DTRACE_INTRUCTION (&stmtAdd);
*/
		}

		dAssert (m_parent);
		#ifdef D_USE_COMPLEX_ADRESSING_MODE

			// emit an indirect addressing mode
			dTreeAdressStmt& tmp = cil.NewStatement()->GetInfo();
			tmp.m_instruction = dTreeAdressStmt::m_load;
			tmp.m_arg0.m_label = cil.NewTemp();
			tmp.m_arg1.m_label = variable;
			tmp.m_arg2.m_label = result;
			// the size of the integer register in power of two
			tmp.m_extraInformation = 2;
			DTRACE_INTRUCTION (&tmp);
			m_result.m_label = tmp.m_arg0.m_label; 

		#else
			dTreeAdressStmt& dimSize = cil.NewStatement()->GetInfo();
			dimSize.m_instruction = dTreeAdressStmt::m_assigment;
			dimSize.m_operator = dTreeAdressStmt::m_mul;
			dimSize.m_arg0.m_label = cil.NewTemp();
			dimSize.m_arg1.m_label = result; 
			dimSize.m_arg2.m_type = dTreeAdressStmt::m_intConst;
			dimSize.m_arg2.m_label = "4"; 
			DTRACE_INTRUCTION (&dimSize);

			dAssert (m_parent);
			// emit an indirect addressing mode
			dTreeAdressStmt& tmp = cil.NewStatement()->GetInfo();
			tmp.m_instruction = dTreeAdressStmt::m_load;
			tmp.m_arg0.m_label = cil.NewTemp();
			tmp.m_arg1.m_label = variable;
			tmp.m_arg2.m_label = dimSize.m_arg0.m_label;
			DTRACE_INTRUCTION (&tmp);
			m_result.m_label = tmp.m_arg0.m_label; 
	   #endif


	} else {
		//m_result.m_label = m_name;
		m_result.m_label = variable;
	}
}


dDAGExpressionNodeVariable* dDAGExpressionNodeVariable::FindLeftVariable()
{
	return this;
}
