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

#ifndef __dDAGFunctionStatementWHILE_H__
#define __dDAGFunctionStatementWHILE_H__

#include "dDAG.h"
#include "dDAGFunctionStatementFlow.h"

class dDAGExpressionNode;

class dDAGFunctionStatementWHILE: public dDAGFunctionStatementFlow
{
	public:
	dDAGFunctionStatementWHILE(dList<dDAG*>& allNodes, dDAGExpressionNode* const expression, dDAGFunctionStatement* const stmt);
	~dDAGFunctionStatementWHILE();

	virtual dDAGFunctionStatement* const GetPostFixStatement() const;

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dDAGRtti(dDAGFunctionStatementFlow);

	dDAGExpressionNode* m_expression;
};


#endif