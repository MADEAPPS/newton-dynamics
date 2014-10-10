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
#include "dDAGFunctionNode.h"
#include "dDAGParameterNode.h"
#include "dDAGExpressionNode.h"
#include "dDAGScopeBlockNode.h"


dInitRtti(dDAGParameterNode);

dDAGParameterNode::dDAGParameterNode(dList<dDAG*>& allNodes, const dString& name, const dString& modifiers)
	:dDAGFunctionStatement(allNodes)
	,m_type(NULL)
{
	m_name = name;
	m_isFinal = modifiers.Find ("final") >= 0;
	m_isStatic = modifiers.Find ("static") >= 0;
	m_isPublic = modifiers.Find ("public") >= 0;
}


dDAGParameterNode::~dDAGParameterNode(void)
{
}


void dDAGParameterNode::SetType(dDAGTypeNode* const type)
{
	m_type = type;
}


dDAGTypeNode* dDAGParameterNode::GetType() const 
{
	return m_type;
}

void dDAGParameterNode::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;

	dDAGScopeBlockNode* const scope = GetScope();
	if (scope) {
		scope->AddVariable (m_name, m_type->GetArgType());
	}
	m_type->ConnectParent(this);
}


void dDAGParameterNode::CompileCIL(dCIL& cil)  
{
	dAssert (0);
/*
	dDAGScopeBlockNode* const scope = GetScope();
	dTree<dCILInstr::dArg, dString>::dTreeNode* const varNameNode = scope->FindVariable(m_name);
	dAssert (varNameNode);

	dCILInstr& fntArg = cil.NewStatement()->GetInfo();
	fntArg.m_instruction = dCILInstr::m_local;
	fntArg.m_arg0 = varNameNode->GetInfo();
	fntArg.m_arg1 = fntArg.m_arg0;
	m_result = fntArg.m_arg0;
	DTRACE_INTRUCTION (&fntArg);
*/
}