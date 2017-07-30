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

#ifndef __dDAGExpressionNodeFunctionCall_H__
#define __dDAGExpressionNodeFunctionCall_H__

#include "dDAG.h"
#include "dDAGExpressionNode.h"



class dDAGExpressionNodeFunctionCall: public dDAGExpressionNode
{
	public:

	dDAGExpressionNodeFunctionCall (dList<dDAG*>& allNodes, const char* const identifier, dDAGExpressionNode* const argumentList);
	~dDAGExpressionNodeFunctionCall(void);

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dList<dDAGExpressionNode*> m_argumentList;
	dDAGRtti(dDAGExpressionNode);
};


#endif