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

#ifndef __dDAGFunctionModifier_H__
#define __dDAGFunctionModifier_H__

#include "dDAG.h"
#include "dLSCstdafx.h"


class dDAGTypeNode;
class dDAGParameterNode;
class dDAGScopeBlockNode;

class dDAGFunctionModifier: public dDAG
{
	public:
	dDAGFunctionModifier(dList<dDAG*>& allNodes);
	~dDAGFunctionModifier(void);

	virtual void CompileCIL(dCIL& cil)  {dAssert (0);}
	virtual void ConnectParent(dDAG* const parent);

	bool m_private;
	bool m_native;
	dDAGRtti(dDAG);
};


#endif