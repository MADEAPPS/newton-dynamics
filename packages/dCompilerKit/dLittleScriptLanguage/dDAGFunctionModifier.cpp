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

#include "dLSCstdafx.h"
#include "dDAG.h"
#include "dDAGFunctionModifier.h"


dInitRtti(dDAGFunctionModifier);

dDAGFunctionModifier::dDAGFunctionModifier(dList<dDAG*>& allNodes)
	:dDAG(allNodes)
	,m_private(false)
	,m_native(false)
{
}


dDAGFunctionModifier::~dDAGFunctionModifier(void)
{
}

void dDAGFunctionModifier::ConnectParent(dDAG* const parent)
{
	m_parent = parent;
}
