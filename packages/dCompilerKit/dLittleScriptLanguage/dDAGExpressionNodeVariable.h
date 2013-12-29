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

#ifndef __dDAGExpressionNodeVariable_H__
#define __dDAGExpressionNodeVariable_H__

#include "dDAG.h"
#include "dDAGExpressionNode.h"

class dDAGTypeNode;
class dDAGDimensionNode;

class dDAGExpressionNodeVariable: public dDAGExpressionNode
{
	public:
	dDAGExpressionNodeVariable (dList<dDAG*>& allNodes, const char* const identifier, dDAGDimensionNode* const expressionDimIndex);
	dDAGExpressionNodeVariable (dList<dDAG*>& allNodes, const dDAGExpressionNodeVariable& copySource);
	~dDAGExpressionNodeVariable(void);

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	void SetType(dDAGTypeNode* const type);
	virtual dDAGExpressionNodeVariable* FindLeftVariable();
	virtual dDAG* Clone (dList<dDAG*>& allNodes) const;

	dDAGTypeNode* m_type;
	dList<dDAGDimensionNode*> m_dimExpressions;
	dDAGRtti(dDAGExpressionNode);
};


#endif