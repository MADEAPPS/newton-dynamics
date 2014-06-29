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

#ifndef __dDAGExpressionNodeConstant_H__
#define __dDAGExpressionNodeConstant_H__

#include "dDAG.h"
#include "dDAGExpressionNode.h"


class dDAGExpressionNodeConstant: public dDAGExpressionNode
{
	public:

	dDAGExpressionNodeConstant (dList<dDAG*>& allNodes, dTreeAdressStmt::dArgType type, const char* const identifier);
	~dDAGExpressionNodeConstant(void);

	virtual dCIL::dReturnValue Evalue(dCIL& cil);
	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dTreeAdressStmt::dArgType m_type;
	dDAGRtti(dDAGExpressionNode);
};

class dDAGExpressionNodeOperatorThisConstant: public dDAGExpressionNodeConstant
{
	public:
	dDAGExpressionNodeOperatorThisConstant (dList<dDAG*>& allNodes);
	virtual void CompileCIL(dCIL& cil);

	dDAGRtti(dDAGExpressionNodeOperatorThisConstant);
};

#endif