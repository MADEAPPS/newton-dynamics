/* Copyright (c) <2003-2016> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dLSCstdafx.h"
#include "dDAG.h"
#include "dDAGFunctionStatement.h"

dInitRtti(dDAGFunctionStatement);
//dString dDAGFunctionStatement::m_loopHeadMetaData	("loopHeader");
//dString dDAGFunctionStatement::m_loopTailMetaData	("loopTail");


dDAGFunctionStatement::dDAGFunctionStatement(dList<dDAG*>& allNodes)
	:dDAG(allNodes)
{
}


dDAGFunctionStatement::~dDAGFunctionStatement()
{
}


void dDAGFunctionStatement::CompileCIL(dCIL& cil)  
{
}

void dDAGFunctionStatement::ConnectParent(dDAG* const parent)  
{
dAssert (0);
	m_parent = parent;
}
