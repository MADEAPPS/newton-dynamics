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

#ifndef __dDAGFunctionStatement_H__
#define __dDAGFunctionStatement_H__

#include "dDAG.h"
#include "dLSCstdafx.h"



class dDAGFunctionStatement: public dDAG
{
	public:
	dDAGFunctionStatement(dList<dDAG*>& allNodes);
	~dDAGFunctionStatement();

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	dDAGRtti(dDAG);
	//static dString m_loopHeadMetaData;
	//static dString m_loopTailMetaData;
};


#endif