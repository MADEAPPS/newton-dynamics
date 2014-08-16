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

	void TranslateToLLVM (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);
	
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

	class dSymbols: public dTree <llvm::Value*, dString>
	{
		public:
		dSymbols ()
			:dTree <llvm::Value*, dString>()
		{
		}
	};


	void CreateLLVMBasicBlocks (llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);


	llvm::Function* CreateLLVMfuntionDeclaration (dSymbols& symbols, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);
	llvm::Value* GetLLVMConstantOrValue (dSymbols& symbols, const dTreeAdressStmt::dArg& arg, llvm::LLVMContext &context);

	void TranslateLLVMBlock (dSymbols& symbols, const LLVMBlockScripBlockPair& llvmBlockPair, llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context);
	void EmitLLVMLocalVariable (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMStoreBase (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMLoadBase (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMAssignment (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);

	void EmitLLVMIf (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMGoto (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMParam (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMCall (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);
	void EmitLLVMReturn (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context);

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
//	dBasicBlocksList m_basicBlocks____; 
	dList<dDAGParameterNode*> m_parameters; 
	LLVMBlockScripBlockPairMap m_blockMap; 
//	llvm::ArrayRef<llvm::Value *> m_paramList;
	dList<llvm::Value*> m_paramList;
	
	dDAGRtti(dDAG);
};


#endif