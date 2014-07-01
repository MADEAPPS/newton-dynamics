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
#include "dDAGTypeNode.h"
#include "dDAGClassNode.h"
#include "dDAGFunctionNode.h"
#include "dDAGParameterNode.h"
#include "dDAGScopeBlockNode.h"
#include "dDAGFunctionModifier.h"
#include "dDAGFunctionStatementReturn.h"

dInitRtti(dDAGFunctionNode);

dDAGFunctionNode::dDAGFunctionNode(dList<dDAG*>& allNodes, dDAGTypeNode* const type, const char* const name, const char* const visivility)
	:dDAG(allNodes)
	,m_isStatic(false)
	,m_isPublic(true)
	,m_isConstructor(false)
	,m_loopLayer(0)
	,m_opertatorThis()
	,m_returnType (type)
	,m_body(NULL)
	,m_modifier(NULL)
	,m_functionStart(NULL)
	,m_basicBlocks()
	,m_parameters() 
{
	m_name = name;

	m_isStatic = strstr (visivility, "static") ? true : false;
	m_isPublic = strstr (visivility, "public") ? true : false;

	if (!m_isStatic) {
		dDAGParameterNode* const operatorThis = new dDAGParameterNode (allNodes, "this", "");
		operatorThis->SetType(new dDAGTypeNode (allNodes, "this"));
		AddParameter(operatorThis);
	}

}


dDAGFunctionNode::~dDAGFunctionNode(void)
{
	dAssert (m_returnType);
}



void dDAGFunctionNode::AddParameter(dDAGParameterNode* const parameter)
{
	dAssert (parameter->IsType(dDAGParameterNode::GetRttiType()));
	m_parameters.Append(parameter);
}

void dDAGFunctionNode::SetBody(dDAGScopeBlockNode* const body)
{
	m_body = body;
}

void dDAGFunctionNode::SetModifier(dDAGFunctionModifier* const modifier)
{
	m_modifier = modifier;
	dAssert (0);
//	m_modifier->AddRef();
}


dDAGParameterNode* dDAGFunctionNode::FindArgumentVariable(const char* const name) const
{
	for (dList<dDAGParameterNode*>::dListNode* node = m_parameters.GetFirst(); node; node = node->GetNext()) {
		dDAGParameterNode* const variable = node->GetInfo();
		if (variable->m_name == name) {
			return variable;
		}
	}
	return NULL;
}

void dDAGFunctionNode::ConnectParent(dDAG* const parent)
{
	m_parent = parent;

	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		m_body->AddVariable (arg->m_name, arg->m_type->m_intrinsicType);
		arg->ConnectParent(this);
	}

	m_body->ConnectParent(this);
	m_returnType->ConnectParent(this);

	if (m_modifier) {
		m_modifier->ConnectParent(this);
	}
}


void dDAGFunctionNode::CompileCIL(dCIL& cil)  
{
	dAssert (m_body);
	dDAGClassNode* const myClass = GetClass();

	cil.ResetTemporaries();
	dString returnVariable (cil.NewTemp());

	dString functionName (myClass->GetFunctionName (m_name.GetStr(), m_parameters));

	dCIL::dListNode* const functionNode = cil.NewStatement();
	m_functionStart = functionNode;

	dTreeAdressStmt& function = functionNode->GetInfo();
	function.m_instruction = dTreeAdressStmt::m_function;
	function.m_arg0.m_label = functionName;
	function.m_arg0.m_type = m_returnType->m_intrinsicType;
	DTRACE_INTRUCTION (&function);


	if (!m_isStatic) {
		dAssert (0);
//		dList<dDAGParameterNode*>::dListNode* const argNode = m_parameters.GetFirst();
//		dDAGParameterNode* const arg = argNode->GetInfo();
//		m_opertatorThis = arg->m_result.m_label;
	}


	// emit the function arguments
	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();

		dTreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
		fntArg.m_instruction = dTreeAdressStmt::m_argument;
		fntArg.m_arg0.m_label = arg->m_name;
		fntArg.m_arg0.m_type = arg->m_type->m_intrinsicType;
		fntArg.m_arg1 = fntArg.m_arg0;
		arg->m_result = fntArg.m_arg0;
		DTRACE_INTRUCTION (&fntArg);
	}

	dTreeAdressStmt& entryPoint = cil.NewStatement()->GetInfo();
	entryPoint.m_instruction = dTreeAdressStmt::m_label;
	entryPoint.m_arg0.m_label = cil.NewLabel();
	DTRACE_INTRUCTION (&entryPoint);


	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dTree<dTreeAdressStmt::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
		dAssert (varNameNode);

		dTreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
		fntArg.m_instruction = dTreeAdressStmt::m_local;
		fntArg.m_arg0 = varNameNode->GetInfo();
		fntArg.m_arg1 = fntArg.m_arg0;
		arg->m_result = fntArg.m_arg1;
		DTRACE_INTRUCTION (&fntArg);
	}


	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dTree<dTreeAdressStmt::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
		dAssert (varNameNode);

		dTreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
		fntArg.m_instruction = dTreeAdressStmt::m_storeBase;
		fntArg.m_arg0 = varNameNode->GetInfo();
		fntArg.m_arg1.m_label = arg->m_name;
		fntArg.m_arg1.m_type = arg->m_type->m_intrinsicType;
		arg->m_result = fntArg.m_arg1;
		DTRACE_INTRUCTION (&fntArg);
	}

	m_body->CompileCIL(cil);
}


void dDAGFunctionNode::BuildBasicBlocks(dCIL& cil, dCIL::dListNode* const functionNode)
{
	// build leading block map table
	m_basicBlocks.RemoveAll();

	// remove redundant jumps
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = functionNode; node; node = nextNode) {
		nextNode = node->GetNext(); 
		const dTreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_goto) {
			if (stmt.m_jmpTarget == nextNode) {
				const dTreeAdressStmt& nextStmt = nextNode->GetInfo();
				dAssert (nextStmt.m_instruction == dTreeAdressStmt::m_label);
				dCIL::dListNode* const prevNode = node->GetPrev();
				const dTreeAdressStmt& prevStmt = prevNode->GetInfo();
				if (prevStmt.m_instruction == dTreeAdressStmt::m_ret) {
					cil.Remove(node);
				}
			}
		}
	}

	// find the root of all basic blocks leaders
	for (dCIL::dListNode* node = functionNode; node; node = node->GetNext()) {
		const dTreeAdressStmt& stmt = node->GetInfo();

		if (stmt.m_instruction == dTreeAdressStmt::m_label) {
			m_basicBlocks.Append(dBasicBlock(node));
		}
	}

	for (dList<dBasicBlock>::dListNode* blockNode = m_basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();

		for (dCIL::dListNode* stmtNode = block.m_begin; !block.m_end && stmtNode; stmtNode = stmtNode->GetNext()) {
			const dTreeAdressStmt& stmt = stmtNode->GetInfo();
			switch (stmt.m_instruction)
			{
				case dTreeAdressStmt::m_if:
				case dTreeAdressStmt::m_goto:
				case dTreeAdressStmt::m_ret:
					block.m_end = stmtNode;
					break;
			}
		} 
	}
}

llvm::Function* dDAGFunctionNode::CreateLLVMfuntionPrototype (dSymbols& symbols, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	std::vector<llvm::Type *> argumentList;
	for (dCIL::dListNode* argNode = m_functionStart->GetNext(); argNode && (argNode->GetInfo().m_instruction != dTreeAdressStmt::m_label); argNode = argNode->GetNext()) {
		const dTreeAdressStmt& stmt = argNode->GetInfo();
		dAssert (stmt.m_instruction == dTreeAdressStmt::m_argument);	
		switch (stmt.m_arg0.m_type)
		{
			case dTreeAdressStmt::m_int:
				argumentList.push_back(llvm::Type::getInt32Ty(context));
				break;

			default:
				dAssert(0);
		}
	}

	const dTreeAdressStmt& functionProto = m_functionStart->GetInfo();
	dAssert (functionProto.m_instruction == dTreeAdressStmt::m_function);	
	llvm::Type* returnTypeVal = NULL;
	switch (functionProto.m_arg0.m_type)
	{
		case dTreeAdressStmt::m_int:
		{
			returnTypeVal = llvm::Type::getInt32Ty(context);
			break;
		}

		default:
			dAssert (0);
	}

	// create the function prototype
	llvm::FunctionType* const funtionParametersAndType = llvm::FunctionType::get (returnTypeVal, argumentList, false);
	llvm::Function* const llvmFunction = llvm::cast<llvm::Function>(module->getOrInsertFunction(functionProto.m_arg0.m_label.GetStr(), funtionParametersAndType));

	// set arguments names.
	llvm::Function::arg_iterator argumnetIter = llvmFunction->arg_begin();  
	for (dCIL::dListNode* argNode = m_functionStart->GetNext(); argNode && (argNode->GetInfo().m_instruction != dTreeAdressStmt::m_label); argNode = argNode->GetNext()) {
		const dTreeAdressStmt& stmt = argNode->GetInfo();
		dAssert (stmt.m_instruction == dTreeAdressStmt::m_argument);	
		switch (stmt.m_arg0.m_type)
		{
			case dTreeAdressStmt::m_int:
			{
				llvm::Argument* const funtionArg = argumnetIter;
				funtionArg->setName (stmt.m_arg0.m_label.GetStr());
				break;
			}

			default:
				dAssert(0);
		}
		symbols.Insert(argumnetIter, stmt.m_arg0.m_label.GetStr());
		argumnetIter ++;
	}

	return llvmFunction;
}

void dDAGFunctionNode::CreateLLVMBasicBlocks (dList<LLVMBlockScripBlockPair>& llvmBlocks, llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	for (dList<dBasicBlock>::dListNode* blockNode = m_basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		dCIL::dListNode* const blockNameNode = block.m_begin;
		const dTreeAdressStmt& blockStmt = blockNameNode->GetInfo();
		dAssert (blockStmt.m_instruction == dTreeAdressStmt::m_label);
		llvm::BasicBlock* const llvmBlock = llvm::BasicBlock::Create(context, blockStmt.m_arg0.m_label.GetStr(), function);
		dAssert (llvmBlock);
		llvmBlocks.Append(LLVMBlockScripBlockPair (llvmBlock, blockNode));
	}
}

void dDAGFunctionNode::EmitLLVMLocalVariable (dSymbols& symbols, const dTreeAdressStmt& stmt, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	switch (stmt.m_arg0.m_type)
	{
		case dTreeAdressStmt::m_int:
		{
			llvm::AllocaInst* const local = new llvm::AllocaInst(llvm::IntegerType::get(context, 8 * sizeof (int)), stmt.m_arg0.m_label.GetStr(), llvmBlock);
			local->setAlignment (sizeof (int));
			symbols.Insert(local, stmt.m_arg0.m_label.GetStr()) ;
			break;
		}

		default:
			dAssert(0);
	}
}

void dDAGFunctionNode::EmitLLVMStoreBase (dSymbols& symbols, const dTreeAdressStmt& stmt, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	dAssert (symbols.Find (stmt.m_arg0.m_label.GetStr()));
	dAssert (symbols.Find (stmt.m_arg1.m_label.GetStr()));
	llvm::Value* const dst = symbols.Find (stmt.m_arg0.m_label.GetStr())->GetInfo();
	llvm::Value* const src = symbols.Find (stmt.m_arg1.m_label.GetStr())->GetInfo();
	switch (stmt.m_arg0.m_type)
	{
		case dTreeAdressStmt::m_int:
		{
			llvm::StoreInst* const local = new llvm::StoreInst(src, dst, false, llvmBlock);
			local->setAlignment(4);
			break;
		}

		default:
			dAssert(0);
	}
}

void dDAGFunctionNode::EmitLLVMLoadBase (dSymbols& symbols, const dTreeAdressStmt& stmt, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	dAssert (symbols.Find (stmt.m_arg1.m_label.GetStr()));
	llvm::Value* const src = symbols.Find (stmt.m_arg1.m_label.GetStr())->GetInfo();
	switch (stmt.m_arg0.m_type)
	{
		case dTreeAdressStmt::m_int:
		{
			llvm::LoadInst* const local = new llvm::LoadInst (src, stmt.m_arg0.m_label.GetStr(), false, llvmBlock);
			local->setAlignment(4);
			symbols.Insert(local, stmt.m_arg0.m_label.GetStr()) ;
			break;
		}

		default:
			dAssert(0);
	}
}


void dDAGFunctionNode::EmitLLVMReturn (dSymbols& symbols, const dTreeAdressStmt& stmt, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	dAssert (symbols.Find (stmt.m_arg0.m_label.GetStr()));
	llvm::Value* const src = symbols.Find (stmt.m_arg0.m_label.GetStr())->GetInfo();
	switch (stmt.m_arg0.m_type)
	{
		case dTreeAdressStmt::m_int:
		{
			llvm::ReturnInst::Create(context, src, llvmBlock);
			break;
		}

		default:
			dAssert(0);
	}
}


void dDAGFunctionNode::TranslateLLVMBlock (dSymbols& symbols, const LLVMBlockScripBlockPair& llvmBlockPair, llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	llvm::BasicBlock* const llvmBlock = llvmBlockPair.m_llvmBlock;
	const dBasicBlock& block = llvmBlockPair.m_blockNode->GetInfo();
	
	dCIL::dListNode* const endNode = block.m_end->GetNext();
	for (dCIL::dListNode* argNode = block.m_begin->GetNext(); argNode != endNode; argNode = argNode->GetNext()) {
		const dTreeAdressStmt& stmt = argNode->GetInfo();
		dAssert (stmt.m_instruction != dTreeAdressStmt::m_label);	

		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_local:
			{
				EmitLLVMLocalVariable (symbols, stmt, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_storeBase:
			{
				EmitLLVMStoreBase (symbols, stmt, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_loadBase:
			{
				EmitLLVMLoadBase (symbols, stmt, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_ret:
			{
				EmitLLVMReturn (symbols, stmt, llvmBlock, context);
				break;
			}


			default:
				dAssert (0);
		}
	} 
}

void dDAGFunctionNode::TranslateToLLVM (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	dSymbols symbols;
	llvm::Function* const llvmFunction = CreateLLVMfuntionPrototype (symbols, cil, module, context);

	dList<LLVMBlockScripBlockPair> llvmBlocks;
	CreateLLVMBasicBlocks (llvmBlocks, llvmFunction, cil, module, context);

	
	for (dList<LLVMBlockScripBlockPair>::dListNode* node = llvmBlocks.GetFirst(); node; node = node->GetNext()) {
		const LLVMBlockScripBlockPair& pair = node->GetInfo();
		TranslateLLVMBlock (symbols, pair, llvmFunction, cil, module, context);
	}
}