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

#ifndef __dDAGExpressionNodeLogicOperator_H__
#define __dDAGExpressionNodeLogicOperator_H__

#include "dDAG.h"
#include "dDAGExpressionNode.h"


class dDAGExpressionNodeLogicOperator: public dDAGExpressionNode
{
	public:

	enum dLogicOperator
	{
		m_or = 0,
		m_and = 1,
	};
	
	dDAGExpressionNodeLogicOperator (dList<dDAG*>& allNodes, dLogicOperator binaryOperator, dDAGExpressionNode* const expressionA, dDAGExpressionNode* const expressionB);
	~dDAGExpressionNodeLogicOperator(void);


//	dTreeAdressStmt::dArg SetArgument(dDAGExpressionNode* const expresion);

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dLogicOperator m_operator;
	dDAGExpressionNode* m_expressionA;
	dDAGExpressionNode* m_expressionB;

	dDAGRtti(dDAGExpressionNode);
};


#endif