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

#ifndef __dDAGExpressionClassVariable_H__
#define __dDAGExpressionClassVariable_H__

#include "dDAG.h"
#include "dLSCstdafx.h"
#include "dDAGExpressionNode.h"

class dDAGExpressionNodeVariable;

class dDAGExpressionClassVariable: public dDAGExpressionNode
{
	public:
	dDAGExpressionClassVariable (dList<dDAG*>& allNodes, dDAGExpressionNode* const expression);
	~dDAGExpressionClassVariable ();

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dDAGExpressionNode* m_expression;
	dDAGExpressionNodeVariable* m_variable;

	dDAGRtti(dDAGExpressionNode);
};


#endif