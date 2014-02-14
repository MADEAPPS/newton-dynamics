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


void dDAGParameterNode::ConnectParent(dDAG* const parent)  
{
	m_parent = parent;
	m_type->ConnectParent(this);
//	if (m_initializationExp) {
//		m_initializationExp->ConnectParent(this);
//	}
}


void dDAGParameterNode::CompileCIL(dCIL& cil)  
{
	dAssert (0);
	/*

	char text[512];
	dDAGScopeBlockNode* const scope = GetScope();

	sprintf (text, "%s%d%s", D_SCOPE_PREFIX, scope->m_scopeLayer, m_name.GetStr());
	if (scope->m_localVariablesFilter.FindVariable (text)) {
		dTrace (("duplicated local variable\n"));
		dAssert (0);
	}
	m_name = text;
	scope->m_localVariablesFilter.Append(m_name);
*/
}
