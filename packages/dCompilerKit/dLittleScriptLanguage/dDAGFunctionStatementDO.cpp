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
#include "dDAGExpressionNode.h"
#include "dDAGFunctionStatementDO.h"


dInitRtti(dDAGFunctionStatementDO);

dDAGFunctionStatementDO::dDAGFunctionStatementDO(dList<dDAG*>& allNodes, dDAGExpressionNode* const expression, dDAGFunctionStatement* const stmt)
	:dDAGFunctionStatementFlow(allNodes, stmt, expression)
	,m_expression(expression)
{
}


dDAGFunctionStatementDO::~dDAGFunctionStatementDO()
{
}

void dDAGFunctionStatementDO::ConnectParent(dDAG* const parent)
{
	dDAGFunctionStatementFlow::ConnectParent(parent);
}


void dDAGFunctionStatementDO::CompileCIL(dCIL& cil)  
{
	dAssert (0);
	/*

	CompileCILLoopBody(cil, NULL);
*/
}