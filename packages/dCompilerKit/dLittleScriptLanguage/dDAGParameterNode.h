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

#ifndef __dDAGParameterNode_H__
#define __dDAGParameterNode_H__

#include "dDAG.h"
#include "dDAGFunctionStatement.h"

class dDAGTypeNode;
class dDAGExpressionNode;
class dDAGScopeBlockNode;

class dDAGParameterNode: public dDAGFunctionStatement
{
	public:
	dDAGParameterNode (dList<dDAG*>& allNodes, const dString& name, const dString& modifiers);
	~dDAGParameterNode(void);

	void SetType(dDAGTypeNode* const type);

	virtual void CompileCIL(dCIL& cil); 
	virtual void ConnectParent(dDAG* const parent);

	bool m_isFinal;
	bool m_isPublic;
	bool m_isStatic;

	dDAGTypeNode* m_type;
	dDAGRtti(dDAGFunctionStatement);
};


#endif