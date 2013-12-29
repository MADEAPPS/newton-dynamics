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

#ifndef __dDAGDimensionNode_H__
#define __dDAGDimensionNode_H__

#include "dDAG.h"
#include "dLSCstdafx.h"

class dDAGExpressionNode;


class dDAGDimensionNode: public dDAG
{
	public:
	dDAGDimensionNode(dList<dDAG*>& allNodes, dDAGExpressionNode* const size);
	dDAGDimensionNode(dList<dDAG*>& allNodes, const dDAGDimensionNode& copy);
	~dDAGDimensionNode(void);

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);
	virtual dDAG* Clone (dList<dDAG*>& allNodes) const;

	dString m_arraySize;
	dDAGExpressionNode* m_dimExp;
	dDAGDimensionNode* m_next;
	dDAGRtti(dDAG);
};


#endif