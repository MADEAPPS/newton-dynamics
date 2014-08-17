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

#ifndef __dDAG_h_
#define __dDAG_h_

#include "dCILstdafx.h"
#include "dLSCstdafx.h"

#ifdef _MSC_VER
#pragma warning (disable: 4100) // warning C4100: unreferenced formal parameter
#endif


class dDAGClassNode;
class dDAGFunctionNode;
class dDAGScopeBlockNode;

#define DGDAG_API	

#define dDAGRtti(x)							dAddRtti(x,DGDAG_API)
#define dDAGRttiRootClassSupportDeclare(x)	dRttiRootClassSupportDeclare(x,DGDAG_API)

class dDAG
{
	public:
	class dLLVMSymbols: public dTree <llvm::Value*, dString>
	{
		public:
		dLLVMSymbols ()
			:dTree <llvm::Value*, dString>()
		{
		}
	};


	dDAG(dList<dDAG*>& allNodes);
	virtual ~dDAG(void);
	
	virtual void CompileCIL(dCIL& cil)  {dAssert (0);}
	virtual void ConnectParent(dDAG* const parent) {dAssert (0);}
	virtual dDAG* Clone (dList<dDAG*>& allNodes) const {dAssert (0); return NULL;}

	dDAGClassNode* GetClass() const;
	dDAGScopeBlockNode* GetScope() const;
	dDAGFunctionNode* GetFunction() const;
	virtual dTree<dTreeAdressStmt::dArg, dString>::dTreeNode* FindLocalVariable(const dString& name) const;

	dTreeAdressStmt::dArg LoadLocalVariable (dCIL& cil, const dTreeAdressStmt::dArg& arg) const;
	
	dString m_name;
	dTreeAdressStmt::dArg m_result;
	dDAG* m_next;
	dDAG* m_parent;
	dList<dDAG*>::dListNode* m_myListNode;

	static dString m_scopePrefix;
	static dString m_prototypeSeparator;

	dDAGRttiRootClassSupportDeclare(dDAG);
	
};


#endif