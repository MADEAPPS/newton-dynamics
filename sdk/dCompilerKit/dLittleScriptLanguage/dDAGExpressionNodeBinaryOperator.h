/* Copyright (c) <2003-2016> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __dDAGExpressionNodeBinaryOperator_H__
#define __dDAGExpressionNodeBinaryOperator_H__

#include "dDAG.h"
#include "dDAGExpressionNode.h"

class dDAGExpressionNodeBinaryOperator: public dDAGExpressionNode
{
	public:

	enum dBinaryOperator
	{
		m_add = '+',
		m_sub,
		m_mul,
		m_div,
		m_mod,
		m_less,
		m_greather,
		m_lessEqual,
		m_greatherEqual,
		m_identical,
		m_different,
	};
	
	dDAGExpressionNodeBinaryOperator (dList<dDAG*>& allNodes, dBinaryOperator binaryOperator, dDAGExpressionNode* const expressionA, dDAGExpressionNode* const expressionB);
	~dDAGExpressionNodeBinaryOperator(void);


	virtual dCIL::dReturnValue Evalue(const dDAGFunctionNode* const function);
	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	//void PromoteTypes (dCIL::dReturnValue& typeA, dCIL::dReturnValue& typeB) const;
	dCILInstr::dArgType PromoteTypes (const dCILInstr::dArgType typeA, const dCILInstr::dArgType typeB) const;

	dBinaryOperator m_operator;
	dDAGExpressionNode* m_expressionA;
	dDAGExpressionNode* m_expressionB;

	dDAGRtti(dDAGExpressionNode);
};


#endif