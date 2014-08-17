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

#ifndef __dDAGExpressionNodeAssigment_H__
#define __dDAGExpressionNodeAssigment_H__

#include "dDAG.h"
#include "dDAGExpressionNode.h"
#include "dDAGExpressionNodeVariable.h"
#include "dDAGExpressionNodeBinaryOperator.h"

class dDAGExpressionNodeAssigment: public dDAGExpressionNode
{
	public:
	dDAGExpressionNodeAssigment(dList<dDAG*>& allNodes, dDAGExpressionNodeVariable* const leftVariable, dDAGExpressionNode* const expression);
	~dDAGExpressionNodeAssigment();

	virtual dCIL::dReturnValue Evalue(const dDAGFunctionNode* const function);
	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);
	virtual dDAGExpressionNodeVariable* FindLeftVariable();

	dDAGExpressionNode* m_expression;
	dDAGExpressionNodeVariable* m_leftVariable;
	dDAGRtti(dDAGExpressionNode);
};


#endif