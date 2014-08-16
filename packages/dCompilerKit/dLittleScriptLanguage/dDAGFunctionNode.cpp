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
//	,m_basicBlocks____()
	,m_parameters() 
	,m_blockMap()
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

	dTreeAdressStmt& entryPoint = cil.NewStatement()->GetInfo();
	entryPoint.m_instruction = dTreeAdressStmt::m_label;
	entryPoint.m_arg0.m_label = cil.NewLabel();
	DTRACE_INTRUCTION (&entryPoint);

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
/*
	dTreeAdressStmt& zeroLocal = cil.NewStatement()->GetInfo();
	zeroLocal.m_instruction = dTreeAdressStmt::m_local;
	zeroLocal.m_arg0.m_label = cil.NewTemp();;
	zeroLocal.m_arg0.m_type = dTreeAdressStmt::m_int;
	zeroLocal.m_arg1 = zeroLocal.m_arg0;
	DTRACE_INTRUCTION (&zeroLocal);

	dTreeAdressStmt& zero = cil.NewStatement()->GetInfo();
	zero.m_instruction = dTreeAdressStmt::m_storeBase;
	zero.m_arg1.m_label = "0";
	zero.m_arg1.m_type = dTreeAdressStmt::m_int;
	zero.m_arg0 = zeroLocal.m_arg1;
	DTRACE_INTRUCTION (&zero);
	m_zero = zeroLocal.m_arg0;
*/

	m_body->CompileCIL(cil);
}

/*
void dDAGFunctionNode::ClearBasicBlocks ()
{
	m_basicBlocks.RemoveAll();
}

void dDAGFunctionNode::BuildBasicBlocks(dCIL& cil, dCIL::dListNode* const functionNode)
{
	// build leading block map table
	void ClearBasicBlocks ();

	// remove redundant jumps
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = functionNode; node; node = nextNode) {
		nextNode = node->GetNext(); 
		const dTreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_goto) {
			dCIL::dListNode* const prevNode = node->GetPrev();
			const dTreeAdressStmt& prevStmt = prevNode->GetInfo();
			if (prevStmt.m_instruction == dTreeAdressStmt::m_ret) {
				cil.Remove(node);
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
*/

llvm::Function* dDAGFunctionNode::CreateLLVMfuntionDeclaration (dSymbols& symbols, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	std::vector<llvm::Type *> argumentList;
	for (dCIL::dListNode* argNode = m_functionStart->GetNext()->GetNext(); argNode && (argNode->GetInfo().m_instruction != dTreeAdressStmt::m_label); argNode = argNode->GetNext()) {
		const dTreeAdressStmt& stmt = argNode->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_argument) {

			switch (stmt.m_arg0.m_type)
			{
				case dTreeAdressStmt::m_int:
					argumentList.push_back(llvm::Type::getInt32Ty(context));
					break;

				default:
					dAssert(0);
			}
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
	for (dCIL::dListNode* argNode = m_functionStart->GetNext()->GetNext(); argNode && (argNode->GetInfo().m_instruction != dTreeAdressStmt::m_label); argNode = argNode->GetNext()) {
		const dTreeAdressStmt& stmt = argNode->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_argument)
		{
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
	}

	return llvmFunction;
}


llvm::Value* dDAGFunctionNode::GetLLVMConstantOrValue (dSymbols& symbols, const dTreeAdressStmt::dArg& arg, llvm::LLVMContext &context)
{
	dSymbols::dTreeNode* node = symbols.Find (arg.m_label.GetStr());
	if (!node) {
		llvm::Value* value = NULL;
		switch (arg.m_type) 
		{
			case dTreeAdressStmt::m_constInt:
				value = llvm::ConstantInt::get(context, llvm::APInt(32, llvm::StringRef(arg.m_label.GetStr()), 10));

				break;

			default:
				dAssert (0);
		}

		node = symbols.Insert(value, arg.m_label);
	}

	return node->GetInfo();

}


void dDAGFunctionNode::CreateLLVMBasicBlocks (llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	dBasicBlocksList basicBlocks (cil, m_functionStart);
	for (dList<dBasicBlock>::dListNode* blockNode = basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		dCIL::dListNode* const blockNameNode = block.m_begin;
		const dTreeAdressStmt& blockStmt = blockNameNode->GetInfo();
		dAssert (blockStmt.m_instruction == dTreeAdressStmt::m_label);
		llvm::BasicBlock* const llvmBlock = llvm::BasicBlock::Create(context, blockStmt.m_arg0.m_label.GetStr(), function);
		dAssert (llvmBlock);
		m_blockMap.Append(LLVMBlockScripBlockPair (llvmBlock, blockNode));
	}
}



void dDAGFunctionNode::EmitLLVMLocalVariable (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
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

void dDAGFunctionNode::EmitLLVMStoreBase (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (symbols.Find (stmt.m_arg0.m_label.GetStr()));
//	dAssert (symbols.Find (stmt.m_arg1.m_label.GetStr()));
	llvm::Value* const dst = symbols.Find (stmt.m_arg0.m_label.GetStr())->GetInfo();
	llvm::Value* const src = GetLLVMConstantOrValue (symbols, stmt.m_arg1, context);;
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

void dDAGFunctionNode::EmitLLVMLoadBase (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
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


void dDAGFunctionNode::EmitLLVMReturn (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
//	dAssert (symbols.Find (stmt.m_arg0.m_label.GetStr()));
//	llvm::Value* const src = symbols.Find (stmt.m_arg0.m_label.GetStr())->GetInfo();
	llvm::Value* const src = GetLLVMConstantOrValue (symbols, stmt.m_arg0, context);
	switch (stmt.m_arg0.m_type)
	{
		case dTreeAdressStmt::m_int:
		case dTreeAdressStmt::m_constInt:
		{
			llvm::ReturnInst::Create(context, src, llvmBlock);
			break;
		}

		default:
			dAssert(0);
	}
}


void dDAGFunctionNode::EmitLLVMIf (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (stmt.m_operator == dTreeAdressStmt::m_nothing);
	dAssert (symbols.Find (stmt.m_arg0.m_label.GetStr()));

	llvm::Value* const arg0 = symbols.Find (stmt.m_arg0.m_label.GetStr())->GetInfo();
	llvm::BasicBlock* const thenBlock = m_blockMap.Find (stmt.m_trueTargetJump);
	llvm::BasicBlock* const continueBlock = m_blockMap.Find(stmt.m_falseTargetJump);
	dAssert (thenBlock);
	dAssert (continueBlock);
	dAssert (stmt.m_arg0.m_type == dTreeAdressStmt::m_int);
	llvm::BranchInst::Create (thenBlock, continueBlock, arg0, llvmBlock);
}

void dDAGFunctionNode::EmitLLVMGoto (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (stmt.m_operator == dTreeAdressStmt::m_nothing);
	llvm::BasicBlock* const targetBlock = m_blockMap.Find (stmt.m_trueTargetJump);
	dAssert (targetBlock);
	llvm::BranchInst::Create (targetBlock, llvmBlock);
}

void dDAGFunctionNode::EmitLLVMParam (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
	llvm::Value* const src = GetLLVMConstantOrValue (symbols, stmt.m_arg0, context);
	m_paramList.Append(src);
}

void dDAGFunctionNode::EmitLLVMCall (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (stmt.m_operator == dTreeAdressStmt::m_nothing);

	dAssert (symbols.Find (stmt.m_arg1.m_label.GetStr()));
	llvm::Value* const function = symbols.Find (stmt.m_arg1.m_label.GetStr())->GetInfo();

	int count = 0;
	llvm::Value* buffer[128];
	for (dList<llvm::Value*>::dListNode* node = m_paramList.GetFirst(); node; node = node->GetNext()) {
		buffer[count] = node->GetInfo();
		count ++;
	}

	llvm::ArrayRef<llvm::Value *> paramList (buffer, count);

	llvm::CallInst* const call = llvm::CallInst::Create(function, paramList, stmt.m_arg0.m_label.GetStr(), llvmBlock);
	call->setCallingConv(llvm::CallingConv::C);
	call->setTailCall (true);
	llvm::AttributeSet callAttrib;
	call->setAttributes (callAttrib);
	symbols.Insert(call, stmt.m_arg0.m_label.GetStr()) ;

	m_paramList.RemoveAll();
}


void dDAGFunctionNode::EmitLLVMAssignment (dSymbols& symbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dTreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (stmt.m_operator != dTreeAdressStmt::m_nothing);
	dAssert (symbols.Find (stmt.m_arg1.m_label.GetStr()));
	llvm::Value* const arg1 = symbols.Find (stmt.m_arg1.m_label.GetStr())->GetInfo();
	llvm::Value* const arg2 = GetLLVMConstantOrValue (symbols, stmt.m_arg2, context);

	llvm::Value* arg0 = NULL;
	switch (stmt.m_operator)
	{
		case dTreeAdressStmt::m_identical:
		{
			dAssert (stmt.m_arg1.m_type == dTreeAdressStmt::m_int);
			arg0 = new llvm::ICmpInst (*llvmBlock, llvm::ICmpInst::ICMP_EQ, arg1, arg2, stmt.m_arg0.m_label.GetStr());
			break;
		}

		case dTreeAdressStmt::m_sub:
		{
			dAssert (stmt.m_arg1.m_type == dTreeAdressStmt::m_int);
			arg0 = llvm::BinaryOperator::Create (llvm::Instruction::Sub, arg1, arg2, stmt.m_arg0.m_label.GetStr(), llvmBlock);
			break;
		}

		default:
			dAssert(0);
	}
	symbols.Insert(arg0, stmt.m_arg0.m_label.GetStr()) ;
}

void dDAGFunctionNode::TranslateLLVMBlock (dSymbols& symbols, const LLVMBlockScripBlockPair& llvmBlockPair, llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	llvm::BasicBlock* const llvmBlock = llvmBlockPair.m_llvmBlock;
	const dBasicBlock& block = llvmBlockPair.m_blockNode->GetInfo();
	
	dCIL::dListNode* const endNode = block.m_end->GetNext();
	for (dCIL::dListNode* argNode = block.m_begin->GetNext(); argNode != endNode; argNode = argNode->GetNext()) {
		const dTreeAdressStmt& stmt = argNode->GetInfo();
DTRACE_INTRUCTION (&stmt);
		dAssert (stmt.m_instruction != dTreeAdressStmt::m_label);	

		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_local:
			{
				EmitLLVMLocalVariable (symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_storeBase:
			{
				EmitLLVMStoreBase (symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_loadBase:
			{
				EmitLLVMLoadBase (symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_ret:
			{
				EmitLLVMReturn (symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				EmitLLVMAssignment (symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				EmitLLVMIf (symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_goto:
			{
				EmitLLVMGoto (symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_param:
			{
				EmitLLVMParam(symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_call:
			{
				EmitLLVMCall (symbols, argNode, llvmBlock, context);
				break;
			}

			case dTreeAdressStmt::m_argument:
				break;

			default:
				dAssert (0);
		}
	} 
}

void dDAGFunctionNode::TranslateToLLVM (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	dSymbols symbols;
	llvm::Function* const llvmFunction = CreateLLVMfuntionDeclaration (symbols, cil, module, context);

	CreateLLVMBasicBlocks (llvmFunction, cil, module, context);

//	cil.Trace();
	for (LLVMBlockScripBlockPairMap::dListNode* node = m_blockMap.GetFirst(); node; node = node->GetNext()) {
		const LLVMBlockScripBlockPair& pair = node->GetInfo();
		TranslateLLVMBlock (symbols, pair, llvmFunction, cil, module, context);
	}

    // Validate the generated code, checking for consistency.
    dAssert (!llvm::verifyFunction(*llvmFunction));

    cil.Optimize (llvmFunction);
}