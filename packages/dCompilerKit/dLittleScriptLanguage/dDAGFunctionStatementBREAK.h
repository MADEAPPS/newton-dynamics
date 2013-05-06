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

#ifndef __dDAGFunctionStatementBREAK_H__
#define __dDAGFunctionStatementBREAK_H__

#include "dDAG.h"
#include "dDAGFunctionStatement.h"
#include "dDAGExpressionNode.h"


class dDAGFunctionStatementBREAK: public dDAGFunctionStatement
{
	public:
	dDAGFunctionStatementBREAK(dList<dDAG*>& allNodes);
	~dDAGFunctionStatementBREAK();

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dAddRtti(dDAGFunctionStatement);
};


#endif