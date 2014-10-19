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

#ifndef __dDAGFunctionStatementFlow_H__
#define __dDAGFunctionStatementFlow_H__

#include "dDAG.h"
#include "dDAGFunctionStatement.h"

class dDAGExpressionNode;

class dDAGFunctionStatementFlow: public dDAGFunctionStatement
{
	public:
	dDAGFunctionStatementFlow(dList<dDAG*>& allNodes, dDAGFunctionStatement* const loopBodyStmt, dDAGExpressionNode* const testExpression);
	~dDAGFunctionStatementFlow();

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	void BackPatch (dCIL& cil);

	virtual dDAGFunctionStatement* const GetPostFixStatement() const = 0;
	//void OpenPreHeaderBlock(dCIL& cil);
	//void ClosePreHeaderBlock(dCIL& cil);
//	dCIL::dListNode* CompileCILLoopBody (dCIL& cil, dCILInstrConditional* const preConditinalTest, dCILInstrLabel* const loopStart, dDAGFunctionStatement* const posFixStmt);

	dDAGRtti(dDAGFunctionStatement);

	//dString m_currentBreakLabel;
	//dString m_currentContinueLabel;
	//dCIL::dListNode* m_continueTarget;
	//dCIL::dListNode* m_backPatchStart;
	dDAGExpressionNode* m_testExpression;
	dDAGFunctionStatement* m_loopBodyStmt;
};


#endif