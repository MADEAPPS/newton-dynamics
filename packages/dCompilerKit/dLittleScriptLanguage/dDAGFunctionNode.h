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

#ifndef __dDAGFunctionNode_H__
#define __dDAGFunctionNode_H__

#include "dDAG.h"
#include "dLSCstdafx.h"

class dDAGTypeNode;
class dDAGParameterNode;
class dDAGScopeBlockNode;
class dDAGFunctionModifier;

class dDAGFunctionNode: public dDAG
{
	public:

	dDAGFunctionNode(dList<dDAG*>& allNodes, dDAGTypeNode* const type, const char* const name, const char* const visibility);
	~dDAGFunctionNode(void);

	//void ClearBasicBlocks ();
	//void BuildBasicBlocks(dCIL& cil, dCIL::dListNode* const functionNode);
	
	void AddParameter(dDAGParameterNode* const parameter);
	void SetBody(dDAGScopeBlockNode* const body);
	void SetModifier(dDAGFunctionModifier* const modifier);

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);
	dDAGParameterNode* FindArgumentVariable(const char* const name) const;

	void Optimize (dCIL& cil);
	void ConvertToTarget (dCIL& cil);


	private:
	bool RemoveRedundantJumps (dCIL& cil);
	

	public:
	bool m_isStatic;
	bool m_isPublic;
	bool m_isConstructor;
	int m_loopLayer;
	dString m_opertatorThis;
	dDAGTypeNode* m_returnType;
	dDAGScopeBlockNode* m_body;
	dDAGFunctionModifier* m_modifier;
	dCIL::dListNode* m_functionStart;
	dList<dDAGParameterNode*> m_parameters; 
	dBasicBlocksGraph m_basicBlocks;

	dDAGRtti(dDAG);
};


#endif