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

#ifndef __dDAGExpressionNodePrefixPostfix_H__
#define __dDAGExpressionNodePrefixPostfix_H__

#include "dDAG.h"
#include "dLSCstdafx.h"
#include "dDAGExpressionNode.h"


class dDAGExpressionNodePrefixPostfix: public dDAGExpressionNode
{
	public:
	dDAGExpressionNodePrefixPostfix (dList<dDAG*>& allNodes, dDAGExpressionNode* const expression, bool isPrefix, bool isIncrement);
	~dDAGExpressionNodePrefixPostfix();

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	bool m_isPrefix;
	bool m_isIncrement;
	dDAGExpressionNode* m_expression;
	dDAGRtti(dDAGExpressionNode);
};


#endif