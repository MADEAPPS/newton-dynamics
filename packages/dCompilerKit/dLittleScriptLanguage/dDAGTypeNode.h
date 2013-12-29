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

#ifndef __dDAGTypeNode_H__
#define __dDAGTypeNode_H__

#include "dDAG.h"
#include "dLSCstdafx.h"


class dDAGDimensionNode;

class dDAGTypeNode: public dDAG
{
	public:
	dDAGTypeNode(dList<dDAG*>& allNodes, const char* const type);
	dDAGTypeNode(dList<dDAG*>& allNodes, const dDAGTypeNode& copySource);
	~dDAGTypeNode(void);

	void AddDimensions (dDAGDimensionNode* const dimList);

	virtual void CompileCIL(dCIL& cil)  {_ASSERTE (0);}
	virtual void ConnectParent(dDAG* const parent);
	virtual dDAG* Clone (dList<dDAG*>& allNodes) const;

	dList<dDAGDimensionNode*> m_dimensions;

	dDAGRtti(dDAG);
};


#endif