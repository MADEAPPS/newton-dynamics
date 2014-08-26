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

	void TranslateToLLVM (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols);
	void AddLLVMGlobalSymbols (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols);
	
	private:
	class LLVMBlockScripBlockPair
	{
		public:
		LLVMBlockScripBlockPair (llvm::BasicBlock* const llvmBlock, dBasicBlocksList::dListNode* const node)
			:m_llvmBlock(llvmBlock)
			,m_blockNode(node)
		{
		}

		llvm::BasicBlock* m_llvmBlock;
		dBasicBlocksList::dListNode* m_blockNode;
	};

	class LLVMBlockScripBlockPairMap: public dList<LLVMBlockScripBlockPair>
	{
		public:
		LLVMBlockScripBlockPairMap ()
			:dList<LLVMBlockScripBlockPair>()
		{
		}

		llvm::BasicBlock* Find (dCIL::dListNode* const blockNode) const
		{
			for (dListNode* node = GetFirst(); node; node = node->GetNext())
			{
				const LLVMBlockScripBlockPair& pair = node->GetInfo();
				if (pair.m_blockNode->GetInfo().m_begin == blockNode) {
					return pair.m_llvmBlock;
				}
			}

			dAssert (0);
			return NULL;
		}
	};

	void TranslateLLVMBlock (dLLVMSymbols& localSymbols, const LLVMBlockScripBlockPair& llvmBlockPair, llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols);

	//dString GetLLVMArgName (const dThreeAdressStmt::dArg& arg);
	llvm::Type* GetLLVMType (const dThreeAdressStmt::dArg& arg, llvm::LLVMContext &context);
	
	llvm::Function* CreateLLVMfunctionDeclaration (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);
	llvm::Value* GetLLVMConstantOrValue (dLLVMSymbols& localSymbols, const dThreeAdressStmt::dArg& arg, llvm::LLVMContext &context);
	void CreateLLVMBasicBlocks (llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);

	llvm::Function* EmitLLVMfunction (dLLVMSymbols& localSymbols, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols);
	void EmitLLVMLocalVariable (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);

	void EmitLLVMLoad (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMStore (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMLoadBase (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMStoreBase (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMAssignment (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);

	void EmitLLVMIf (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMGoto (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMParam (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMCall (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols);
	void EmitLLVMReturn (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);

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
	LLVMBlockScripBlockPairMap m_blockMap; 
	dList<llvm::Value*> m_paramList;
	dDAGRtti(dDAG);
};


#endif