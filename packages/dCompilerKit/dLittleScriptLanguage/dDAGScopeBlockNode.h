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

#ifndef __dDAGScopeBlockNode_H__
#define __dDAGScopeBlockNode_H__

#include "dDAG.h"
#include "dDAGFunctionStatement.h"
#include "dDAGExpressionNodeConstant.h"
#include "dDAGExpressionNodeBinaryOperator.h"

class dDAGDimensionNode;
class dDAGParameterNode;
class dDAGExpressionNode;
class dDAGExpressionNodeVariable;




class dDAGScopeBlockNode: public dDAGFunctionStatement
{
	public:

	dDAGScopeBlockNode(dList<dDAG*>& allNodes);
	~dDAGScopeBlockNode(void);

	void AddStatement (dDAGFunctionStatement* const statement);

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);
	void AddSize(int size) ;

	void AddVariable (const dString& name, dThreeAdressStmt::dArgType type);
	virtual dTree<dThreeAdressStmt::dArg, dString>::dTreeNode* FindVariable(const dString& name) const;

	dDAGRtti(dDAGFunctionStatement);
	
	dList<dString> m_allocations;
	dTree<dThreeAdressStmt::dArg, dString> m_localVariables;	
	dList<dDAGFunctionStatement*> m_statementList;
	int m_scopeLayer;
};


#endif