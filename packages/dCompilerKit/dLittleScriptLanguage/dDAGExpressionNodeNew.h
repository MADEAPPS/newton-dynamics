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

#ifndef __dDAGExpressionNodeNew_H__
#define __dDAGExpressionNodeNew_H__

#include "dDAG.h"
#include "dDAGExpressionNode.h"

class dDAGDimensionNode;

class dDAGExpressionNodeNew: public dDAGExpressionNode
{
	public:
	dDAGExpressionNodeNew (dList<dDAG*>& allNodes, const char* const typeName, dDAGDimensionNode* const dimension);
	~dDAGExpressionNodeNew(void);

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dDAGDimensionNode* m_dimension;
	dAddRtti(dDAGExpressionNode);
};


#endif