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

#ifndef __dDAGFunctionStatementCase_H__
#define __dDAGFunctionStatementCase_H__

#include "dDAG.h"
#include "dLSCstdafx.h"
#include "dDAGFunctionStatement.h"


class dDAGFunctionStatementCase: public dDAGFunctionStatement
{
	public:
	dDAGFunctionStatementCase(dList<dDAG*>& allNodes, const char* const nameID, dDAGFunctionStatement* const childStatement);
	~dDAGFunctionStatementCase();

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dString m_nameId;
	dString m_entryLabel;
	dList<dDAGFunctionStatement*> m_statementList;
	
	dDAGRtti(dDAGFunctionStatement);
};


#endif