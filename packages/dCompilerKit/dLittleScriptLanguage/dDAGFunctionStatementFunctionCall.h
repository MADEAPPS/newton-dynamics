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

#ifndef __dDAGFunctionStatementFunctionCall_H__
#define __dDAGFunctionStatementFunctionCall_H__

#include "dDAG.h"
#include "dDAGFunctionStatement.h"
#include "dDAGExpressionNode.h"

class dDAGExpressionNodeFunctionCall;

class dDAGFunctionStatementFunctionCall: public dDAGFunctionStatement
{
	public:
	dDAGFunctionStatementFunctionCall(dList<dDAG*>& allNodes, dDAGExpressionNodeFunctionCall* const function);
	~dDAGFunctionStatementFunctionCall();

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dDAGExpressionNodeFunctionCall* m_function;
	dAddRtti(dDAGFunctionStatement);
};


#endif