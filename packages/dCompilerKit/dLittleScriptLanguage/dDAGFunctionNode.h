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
	class dBasicBlock
	{
		public:
		dBasicBlock (dCIL::dListNode* const begin)
			:m_mark (0)
			,m_begin (begin)
			,m_end(NULL)
		{
		}
		void Trace() const;

		int m_mark;
		dCIL::dListNode* m_begin;
		dCIL::dListNode* m_end;
	};

	class dBasicBlocksList: public dList<dBasicBlock> 
	{
		public:
		dBasicBlocksList()
			:dList<dBasicBlock> ()
		{
		}
/*
		void Trace() const
		{
			#ifdef TRACE_INTERMEDIATE_CODE
				for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
					node->GetInfo().Trace();
				}
			#endif
		}
*/
	};


	dDAGFunctionNode(dList<dDAG*>& allNodes, dDAGTypeNode* const type, const char* const name, const char* const visibility);
	~dDAGFunctionNode(void);

	void BuildBasicBlocks(dCIL& cil, dCIL::dListNode* const functionNode);
	void AddParameter(dDAGParameterNode* const parameter);
	void SetBody(dDAGScopeBlockNode* const body);
	void SetModifier(dDAGFunctionModifier* const modifier);

	virtual void CompileCIL(dCIL& cil);
	virtual void ConnectParent(dDAG* const parent);
	dDAGParameterNode* FindArgumentVariable(const char* const name) const;

	void TranslateToLLVM (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);
	
	private:
	struct LLVMBlockScripBlockPair
	{
		LLVMBlockScripBlockPair (llvm::BasicBlock* const llvmBlock, dBasicBlocksList::dListNode* const node)
			:m_llvmBlock(llvmBlock)
			,m_blockNode(node)
		{
		}
		llvm::BasicBlock* m_llvmBlock;
		dBasicBlocksList::dListNode* m_blockNode;
	};


	class dSymbols: public dTree <llvm::Value*, dString>
	{
		public:
		dSymbols ()
			:dTree <llvm::Value*, dString>()
		{
		}
	};


	void CreateLLVMBasicBlocks (dList<LLVMBlockScripBlockPair>& llvmBlocks, llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);

	llvm::Function* CreateLLVMfuntionPrototype (dSymbols& symbols, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);
	void TranslateLLVMBlock (dSymbols& symbols, const LLVMBlockScripBlockPair& llvmBlockPair, llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);
	void EmitLLVMLocalVariable (dSymbols& symbols, const dTreeAdressStmt& stmt, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMStoreBase (dSymbols& symbols, const dTreeAdressStmt& stmt, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMLoadBase (dSymbols& symbols, const dTreeAdressStmt& stmt, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMReturn (dSymbols& symbols, const dTreeAdressStmt& stmt, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);

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
	dBasicBlocksList m_basicBlocks; 
	dList<dDAGParameterNode*> m_parameters; 


	dDAGRtti(dDAG);
};


#endif