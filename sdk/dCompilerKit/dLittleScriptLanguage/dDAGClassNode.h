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

#ifndef __dDAGClassNode_H__
#define __dDAGClassNode_H__

#include "dDAG.h"
#include "dLSCstdafx.h"

class dDAGTypeNode;
class dDAGFunctionNode;
class dDAGParameterNode;
class dDAGExpressionNode;
class dDAGExpressionClassVariable;

class dDAGClassNode: public dDAG
{
	public:
	dDAGClassNode(dList<dDAG*>& allNodes);
	~dDAGClassNode(void);
	
	void FinalizeImplementation (const dString& visibility, const dString& name, dDAGClassNode* const baseClass);
	
	void AddFunction (dDAGFunctionNode* const functionNode);
	void AddVariable (dDAGExpressionClassVariable* const variable);

	dString GetFunctionName (const dString& functionName, dList<dDAGParameterNode*>& parameterNodeList) const;
	dString GetFunctionName (const dString& functionName, dList<dDAGExpressionNode*>& argumentList) const;
	//dDAGTypeNode* GetFunctionReturnType (const dString& functionName) const;
	
	dDAGFunctionNode* GetCurrentFunction ();
	dDAGFunctionNode* GetFunction (const dString& name) const;
	dDAGParameterNode* FindVariable(const dString& name) const;

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);

	void Optimize (dCIL& cil);
	void ConvertToTarget (dCIL& cil);


	bool m_isFinal;
	bool m_isPublic;
	const dDAGClassNode* m_baseClass;
	dList<dDAGFunctionNode*> m_functionList;
	dList<dDAGFunctionNode*> m_constructors;
	dList<dCIL::dListNode*> m_cilCodeList;
	dList<dDAGExpressionClassVariable*> m_variables;
	dDAGRtti(dDAG);
};


#endif