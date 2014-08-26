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
	,m_parameters() 
	,m_blockMap()
{
	m_name = name;

	m_isStatic = strstr (visivility, "static") ? true : false;
	m_isPublic = strstr (visivility, "public") ? true : false;

	if (!m_isStatic) {
		dAssert (0);
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

	dDAGTypeNode* const type = parameter->GetType();
	m_name += m_prototypeSeparator + type->GetArgType().GetTypeName();
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
		m_body->AddVariable (arg->m_name, arg->m_type->GetArgType());
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

	dString functionName (myClass->GetFunctionName (m_name, m_parameters));

	dCIL::dListNode* const functionNode = cil.NewStatement();
	m_functionStart = functionNode;

	dThreeAdressStmt& function = functionNode->GetInfo();
	function.m_instruction = dThreeAdressStmt::m_function;
	function.m_arg0.m_label = functionName;
	function.m_arg0.SetType (m_returnType->GetArgType());
	DTRACE_INTRUCTION (&function);


	if (!m_isStatic) {
		dAssert (0);
//		dList<dDAGParameterNode*>::dListNode* const argNode = m_parameters.GetFirst();
//		dDAGParameterNode* const arg = argNode->GetInfo();
//		m_opertatorThis = arg->m_result.m_label;
	}

	dThreeAdressStmt& entryPoint = cil.NewStatement()->GetInfo();
	entryPoint.m_instruction = dThreeAdressStmt::m_label;
	entryPoint.m_arg0.m_label = cil.NewLabel();
	DTRACE_INTRUCTION (&entryPoint);

	// emit the function arguments
	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dThreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
		fntArg.m_instruction = dThreeAdressStmt::m_argument;
		fntArg.m_arg0.m_label = arg->m_name;
		fntArg.m_arg0.SetType (arg->m_type->GetArgType());
		fntArg.m_arg1 = fntArg.m_arg0;
		arg->m_result = fntArg.m_arg0;
		DTRACE_INTRUCTION (&fntArg);
	}


	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dTree<dThreeAdressStmt::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
		dAssert (varNameNode);

		dThreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
		fntArg.m_instruction = dThreeAdressStmt::m_local;
		fntArg.m_arg0 = varNameNode->GetInfo();
		fntArg.m_arg1 = fntArg.m_arg0;
		arg->m_result = fntArg.m_arg1;
		DTRACE_INTRUCTION (&fntArg);
	}


	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dTree<dThreeAdressStmt::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
		dAssert (varNameNode);

		dThreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
		fntArg.m_instruction = dThreeAdressStmt::m_storeBase;
		fntArg.m_arg0 = varNameNode->GetInfo();
		fntArg.m_arg1.m_label = arg->m_name;
		fntArg.m_arg1.SetType (arg->GetType()->GetArgType());
		arg->m_result = fntArg.m_arg1;
		DTRACE_INTRUCTION (&fntArg);
	}

	m_body->CompileCIL(cil);
	if (m_returnType->GetArgType().m_intrinsicType == dThreeAdressStmt::m_void) {
		const dThreeAdressStmt& lastInstruction = cil.GetLast()->GetInfo();
		if (lastInstruction.m_instruction != dThreeAdressStmt::m_ret) {
			dThreeAdressStmt& fntArg = cil.NewStatement()->GetInfo();
			fntArg.m_instruction = dThreeAdressStmt::m_ret;
			fntArg.m_arg0.m_label = "";
			fntArg.m_arg0.SetType (m_returnType->GetArgType());
			DTRACE_INTRUCTION (&fntArg);
		}
	}

//	cil.Trace();
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
		const dThreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dThreeAdressStmt::m_goto) {
			dCIL::dListNode* const prevNode = node->GetPrev();
			const dThreeAdressStmt& prevStmt = prevNode->GetInfo();
			if (prevStmt.m_instruction == dThreeAdressStmt::m_ret) {
				cil.Remove(node);
			}
		}
	}

	// find the root of all basic blocks leaders
	for (dCIL::dListNode* node = functionNode; node; node = node->GetNext()) {
		const dThreeAdressStmt& stmt = node->GetInfo();

		if (stmt.m_instruction == dThreeAdressStmt::m_label) {
			m_basicBlocks.Append(dBasicBlock(node));
		}
	}

	for (dList<dBasicBlock>::dListNode* blockNode = m_basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();

		for (dCIL::dListNode* stmtNode = block.m_begin; !block.m_end && stmtNode; stmtNode = stmtNode->GetNext()) {
			const dThreeAdressStmt& stmt = stmtNode->GetInfo();
			switch (stmt.m_instruction)
			{
				case dThreeAdressStmt::m_if:
				case dThreeAdressStmt::m_goto:
				case dThreeAdressStmt::m_ret:
					block.m_end = stmtNode;
					break;
			}
		} 
	}
}
*/


void dDAGFunctionNode::CreateLLVMBasicBlocks (llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	dBasicBlocksList basicBlocks (cil, m_functionStart);
	for (dList<dBasicBlock>::dListNode* blockNode = basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		dCIL::dListNode* const blockNameNode = block.m_begin;
		const dThreeAdressStmt& blockStmt = blockNameNode->GetInfo();
		dAssert (blockStmt.m_instruction == dThreeAdressStmt::m_label);
		llvm::BasicBlock* const llvmBlock = llvm::BasicBlock::Create(context, blockStmt.m_arg0.m_label.GetStr(), function);
		dAssert (llvmBlock);
		m_blockMap.Append(LLVMBlockScripBlockPair (llvmBlock, blockNode));
	}
}

/*
dString dDAGFunctionNode::GetLLVMArgName (const dThreeAdressStmt::dArg& arg)
{
	dString name (stmt.m_arg0.m_label);
	if (arg.m_intrinsicType) {
	}
	switch (arg.m_intrinsicType)
	{
		case dThreeAdressStmt::m_int:
		{
			llvm::Argument* const funtionArg = argumnetIter;
			funtionArg->setName (stmt.m_arg0.m_label.GetStr());
			break;
		}

		default:
			dAssert(0);
}
*/

llvm::Type* dDAGFunctionNode::GetLLVMType (const dThreeAdressStmt::dArg& arg, llvm::LLVMContext &context)
{
	llvm::Type* type = NULL;
	if (arg.m_isPointer) {
		switch (arg.m_intrinsicType)
		{
			case dThreeAdressStmt::m_int:

				type = llvm::Type::getInt32PtrTy(context);
				break;

			case dThreeAdressStmt::m_void:
			{
				return llvm::Type::getVoidTy(context)->getPointerTo();
				break;
			}


			default:
				dAssert(0);
		}

	} else {
		switch (arg.m_intrinsicType)
		{
			case dThreeAdressStmt::m_int:

				type = llvm::Type::getInt32Ty(context);
				break;

			case dThreeAdressStmt::m_void:
			{
				type = llvm::Type::getVoidTy (context);
				break;
			}


			default:
				dAssert(0);
		}
	}

	dAssert (type);
	return type;
}

llvm::Function* dDAGFunctionNode::CreateLLVMfunctionDeclaration (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context)
{
	std::vector<llvm::Type *> argumentList;
	for (dCIL::dListNode* argNode = m_functionStart->GetNext()->GetNext(); argNode && (argNode->GetInfo().m_instruction == dThreeAdressStmt::m_argument); argNode = argNode->GetNext()) {
		const dThreeAdressStmt& stmt = argNode->GetInfo();
		argumentList.push_back (GetLLVMType (stmt.m_arg0, context));
	}

	const dThreeAdressStmt& functionProto = m_functionStart->GetInfo();
	dAssert (functionProto.m_instruction == dThreeAdressStmt::m_function);	
	llvm::Type* const returnTypeVal = GetLLVMType (functionProto.m_arg0, context);

	// create the function prototype
	llvm::FunctionType* const funtionParametersAndType = llvm::FunctionType::get (returnTypeVal, argumentList, false);
	llvm::Function* const llvmFunction = llvm::cast<llvm::Function>(module->getOrInsertFunction(functionProto.m_arg0.m_label.GetStr(), funtionParametersAndType));

	// set arguments names.
	llvm::Function::arg_iterator argumnetIter = llvmFunction->arg_begin();  
	for (dCIL::dListNode* argNode = m_functionStart->GetNext()->GetNext(); argNode && (argNode->GetInfo().m_instruction == dThreeAdressStmt::m_argument); argNode = argNode->GetNext()) {
		const dThreeAdressStmt& stmt = argNode->GetInfo();
		switch (stmt.m_arg0.m_intrinsicType)
		{
			case dThreeAdressStmt::m_int:
			{
				llvm::Argument* const funtionArg = argumnetIter;
				funtionArg->setName (stmt.m_arg0.m_label.GetStr());
				break;
			}

			default:
				dAssert(0);
		}
		argumnetIter ++;
	}

	return llvmFunction;
}



llvm::Value* dDAGFunctionNode::GetLLVMConstantOrValue (dLLVMSymbols& localSymbols, const dThreeAdressStmt::dArg& arg, llvm::LLVMContext &context)
{
	dLLVMSymbols::dTreeNode* node = localSymbols.Find (arg.m_label);
	if (!node) {
		llvm::Value* value = NULL;
		dAssert (!arg.m_isPointer);
		switch (arg.m_intrinsicType) 
		{
			case dThreeAdressStmt::m_constInt:
				value = llvm::ConstantInt::get(context, llvm::APInt(32, llvm::StringRef(arg.m_label.GetStr()), 10));
				break;

			default:
				dAssert (0);
		}

		node = localSymbols.Insert(value, arg.m_label);
	}

	return node->GetInfo();
}

llvm::Function* dDAGFunctionNode::EmitLLVMfunction (dLLVMSymbols& localSymbols, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols)
{
	const dThreeAdressStmt& functionProto = m_functionStart->GetInfo();
	dAssert (globalLLVMSymbols.Find (functionProto.m_arg0.m_label));
	llvm::Function* const llvmFunction = (llvm::Function*) globalLLVMSymbols.Find (functionProto.m_arg0.m_label)->GetInfo();

	// set arguments names.
	dCIL::dListNode* argNode = m_functionStart->GetNext()->GetNext();
	for (llvm::Function::arg_iterator iter (llvmFunction->arg_begin()); iter != llvmFunction->arg_end(); iter ++) {
		dThreeAdressStmt& stmt = argNode->GetInfo();
		dAssert (stmt.m_instruction == dThreeAdressStmt::m_argument);
		const llvm::Argument* const argument = iter;
		const llvm::StringRef& name = argument->getName();
		dAssert (stmt.m_arg0.m_label == name.data());
		localSymbols.Insert(iter, stmt.m_arg0.m_label);
		argNode = argNode->GetNext();
	}

	return llvmFunction;
}


void dDAGFunctionNode::EmitLLVMLocalVariable (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();

	if (stmt.m_arg0.m_isPointer) {
		switch (stmt.m_arg0.m_intrinsicType)
		{
			case dThreeAdressStmt::m_int:
			{
				//llvm::AllocaInst* const local = new llvm::AllocaInst(llvm::IntegerType::get(context, 8 * sizeof (int)), stmt.m_arg0.m_label.GetStr(), llvmBlock);
				llvm::IntegerType* const intType = llvm::IntegerType::get(context, 8 * sizeof (int));
				llvm::AllocaInst* const local = new llvm::AllocaInst(llvm::PointerType::get(intType, 0), stmt.m_arg0.m_label.GetStr(), llvmBlock);
				local->setAlignment (sizeof (int));
				localSymbols.Insert(local, stmt.m_arg0.m_label.GetStr()) ;
				break;
			}

			default:
				dAssert(0);
		}

	} else {
		switch (stmt.m_arg0.m_intrinsicType)
		{
			case dThreeAdressStmt::m_int:
			{
				llvm::AllocaInst* const local = new llvm::AllocaInst(llvm::IntegerType::get(context, 8 * sizeof (int)), stmt.m_arg0.m_label.GetStr(), llvmBlock);
				local->setAlignment (sizeof (int));
				localSymbols.Insert(local, stmt.m_arg0.m_label.GetStr()) ;
				break;
			}

			default:
				dAssert(0);
		}
	}

}

void dDAGFunctionNode::EmitLLVMStoreBase (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (localSymbols.Find (stmt.m_arg0.m_label));
	llvm::Value* const dst = localSymbols.Find (stmt.m_arg0.m_label)->GetInfo();
	llvm::Value* const src = GetLLVMConstantOrValue (localSymbols, stmt.m_arg1, context);

	llvm::StoreInst* const local = new llvm::StoreInst(src, dst, false, llvmBlock);
	local->setAlignment(4);
}

void dDAGFunctionNode::EmitLLVMLoad (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (localSymbols.Find (stmt.m_arg1.m_label));
	dAssert (localSymbols.Find (stmt.m_arg2.m_label));
	dAssert (stmt.m_arg1.m_isPointer);
	dAssert (!stmt.m_arg2.m_isPointer);
	dAssert (stmt.m_arg0.m_intrinsicType == stmt.m_arg1.m_intrinsicType);

	llvm::Value* const src = localSymbols.Find (stmt.m_arg1.m_label)->GetInfo();
	llvm::Value* const index = localSymbols.Find (stmt.m_arg2.m_label)->GetInfo();

	dString baseAdress (stmt.m_arg0.m_label + dCIL::m_variableUndercore);
	llvm::GetElementPtrInst* const load = llvm::GetElementPtrInst::Create (src, index, baseAdress.GetStr(), llvmBlock);
	llvm::LoadInst* const local = new llvm::LoadInst (load, stmt.m_arg0.m_label.GetStr(), false, llvmBlock);
	local->setAlignment(4);
	localSymbols.Insert(local, stmt.m_arg0.m_label.GetStr()) ;

}

void dDAGFunctionNode::EmitLLVMLoadBase (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (localSymbols.Find (stmt.m_arg1.m_label));
	llvm::Value* const src = localSymbols.Find (stmt.m_arg1.m_label)->GetInfo();
	llvm::LoadInst* const local = new llvm::LoadInst (src, stmt.m_arg0.m_label.GetStr(), false, llvmBlock);
	local->setAlignment(4);
	localSymbols.Insert(local, stmt.m_arg0.m_label.GetStr()) ;
}


void dDAGFunctionNode::EmitLLVMReturn (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
dAssert (0);
/*
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
//	dAssert (localSymbols.Find (stmt.m_arg0.m_label));
//	llvm::Value* const src = localSymbols.Find (stmt.m_arg0.m_label)->GetInfo();
	llvm::Value* const src = GetLLVMConstantOrValue (localSymbols, stmt.m_arg0, context);
	switch (stmt.m_arg0.m_type)
	{
		case dThreeAdressStmt::m_int:
		case dThreeAdressStmt::m_constInt:
		{
			llvm::ReturnInst::Create(context, src, llvmBlock);
			break;
		}

		default:
			dAssert(0);
	}
*/
}


void dDAGFunctionNode::EmitLLVMIf (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (stmt.m_operator == dThreeAdressStmt::m_nothing);
	dAssert (localSymbols.Find (stmt.m_arg0.m_label));

	llvm::Value* const arg0 = localSymbols.Find (stmt.m_arg0.m_label)->GetInfo();
	llvm::BasicBlock* const thenBlock = m_blockMap.Find (stmt.m_trueTargetJump);
	llvm::BasicBlock* const continueBlock = m_blockMap.Find(stmt.m_falseTargetJump);
	dAssert (thenBlock);
	dAssert (continueBlock);
	dAssert (stmt.m_arg0.m_intrinsicType == dThreeAdressStmt::m_int);
	llvm::BranchInst::Create (thenBlock, continueBlock, arg0, llvmBlock);
}

void dDAGFunctionNode::EmitLLVMGoto (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (stmt.m_operator == dThreeAdressStmt::m_nothing);
	llvm::BasicBlock* const targetBlock = m_blockMap.Find (stmt.m_trueTargetJump);
	dAssert (targetBlock);
	llvm::BranchInst::Create (targetBlock, llvmBlock);
}

void dDAGFunctionNode::EmitLLVMParam (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
	llvm::Value* const src = GetLLVMConstantOrValue (localSymbols, stmt.m_arg0, context);
	m_paramList.Append(src);
}

void dDAGFunctionNode::EmitLLVMCall (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (stmt.m_operator == dThreeAdressStmt::m_nothing);

	dAssert (globalLLVMSymbols.Find (stmt.m_arg1.m_label));
	llvm::Value* const function = globalLLVMSymbols.Find (stmt.m_arg1.m_label)->GetInfo();

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
	localSymbols.Insert(call, stmt.m_arg0.m_label.GetStr()) ;

	m_paramList.RemoveAll();
}


void dDAGFunctionNode::EmitLLVMAssignment (dLLVMSymbols& localSymbols, dCIL::dListNode* const stmtNode, llvm::BasicBlock* const llvmBlock, llvm::LLVMContext &context)
{
	const dThreeAdressStmt& stmt = stmtNode->GetInfo();
	dAssert (stmt.m_operator != dThreeAdressStmt::m_nothing);
	llvm::Value* const arg1 = GetLLVMConstantOrValue (localSymbols, stmt.m_arg1, context);
	llvm::Value* const arg2 = GetLLVMConstantOrValue (localSymbols, stmt.m_arg2, context);

	llvm::Value* arg0 = NULL;
	switch (stmt.m_operator)
	{
		case dThreeAdressStmt::m_add:
		{
			arg0 = llvm::BinaryOperator::Create (llvm::Instruction::Add, arg1, arg2, stmt.m_arg0.m_label.GetStr(), llvmBlock);
			break;
		}

		case dThreeAdressStmt::m_sub:
		{
			arg0 = llvm::BinaryOperator::Create (llvm::Instruction::Sub, arg1, arg2, stmt.m_arg0.m_label.GetStr(), llvmBlock);
			break;
		}

		case dThreeAdressStmt::m_mul:
		{
			arg0 = llvm::BinaryOperator::Create (llvm::Instruction::Mul, arg1, arg2, stmt.m_arg0.m_label.GetStr(), llvmBlock);
			break;
		}

		case dThreeAdressStmt::m_div:
		{
			arg0 = llvm::BinaryOperator::Create (llvm::Instruction::SDiv, arg1, arg2, stmt.m_arg0.m_label.GetStr(), llvmBlock);
			break;
		}

		case dThreeAdressStmt::m_identical:
		{
			arg0 = new llvm::ICmpInst (*llvmBlock, llvm::ICmpInst::ICMP_EQ, arg1, arg2, stmt.m_arg0.m_label.GetStr());
			break;
		}

		case dThreeAdressStmt::m_lessEqual:
		{
			arg0 = new llvm::ICmpInst (*llvmBlock, llvm::ICmpInst::ICMP_SLE, arg1, arg2, stmt.m_arg0.m_label.GetStr());
			break;
		}


		default:
			dAssert(0);
	}
	localSymbols.Insert(arg0, stmt.m_arg0.m_label.GetStr()) ;
}

void dDAGFunctionNode::TranslateLLVMBlock (dLLVMSymbols& localSymbols, const LLVMBlockScripBlockPair& llvmBlockPair, llvm::Function* const function, dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols)
{
	llvm::BasicBlock* const llvmBlock = llvmBlockPair.m_llvmBlock;
	const dBasicBlock& block = llvmBlockPair.m_blockNode->GetInfo();
	
	dCIL::dListNode* const endNode = block.m_end->GetNext();
	for (dCIL::dListNode* argNode = block.m_begin->GetNext(); argNode != endNode; argNode = argNode->GetNext()) {
		const dThreeAdressStmt& stmt = argNode->GetInfo();
DTRACE_INTRUCTION (&stmt);
		dAssert (stmt.m_instruction != dThreeAdressStmt::m_label);	

		switch (stmt.m_instruction)
		{
			case dThreeAdressStmt::m_local:
			{
				EmitLLVMLocalVariable (localSymbols, argNode, llvmBlock, context);
				break;
			}

			case dThreeAdressStmt::m_load:
			{
				EmitLLVMLoad (localSymbols, argNode, llvmBlock, context);
				break;
			}

			case dThreeAdressStmt::m_loadBase:
			{
				EmitLLVMLoadBase (localSymbols, argNode, llvmBlock, context);
				break;
			}

			case dThreeAdressStmt::m_storeBase:
			{
				EmitLLVMStoreBase (localSymbols, argNode, llvmBlock, context);
				break;
			}


			case dThreeAdressStmt::m_ret:
			{
				EmitLLVMReturn (localSymbols, argNode, llvmBlock, context);
				break;
			}

			case dThreeAdressStmt::m_assigment:
			{
				EmitLLVMAssignment (localSymbols, argNode, llvmBlock, context);
				break;
			}

			case dThreeAdressStmt::m_if:
			{
				EmitLLVMIf (localSymbols, argNode, llvmBlock, context);
				break;
			}

			case dThreeAdressStmt::m_goto:
			{
				EmitLLVMGoto (localSymbols, argNode, llvmBlock, context);
				break;
			}

			case dThreeAdressStmt::m_param:
			{
				EmitLLVMParam(localSymbols, argNode, llvmBlock, context);
				break;
			}

			case dThreeAdressStmt::m_call:
			{
				EmitLLVMCall (localSymbols, argNode, llvmBlock, context, globalLLVMSymbols);
				break;
			}

			case dThreeAdressStmt::m_nop:
			case dThreeAdressStmt::m_argument:
				break;


			default:
				dAssert (0);
		}
	} 
}

void dDAGFunctionNode::AddLLVMGlobalSymbols (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols)
{
	const dThreeAdressStmt& functionProto = m_functionStart->GetInfo();
	llvm::Function* const llvmFunction = CreateLLVMfunctionDeclaration (cil, module, context);
	globalLLVMSymbols.Insert (llvmFunction, functionProto.m_arg0.m_label.GetStr());
}

void dDAGFunctionNode::TranslateToLLVM (dCIL& cil, llvm::Module* const module, llvm::LLVMContext &context, dDAG::dLLVMSymbols& globalLLVMSymbols)
{
	dLLVMSymbols localSymbols;
	llvm::Function* const llvmFunction = EmitLLVMfunction (localSymbols, cil, module, context, globalLLVMSymbols);

	CreateLLVMBasicBlocks (llvmFunction, cil, module, context);

//	cil.Trace();
	for (LLVMBlockScripBlockPairMap::dListNode* node = m_blockMap.GetFirst(); node; node = node->GetNext()) {
		const LLVMBlockScripBlockPair& pair = node->GetInfo();
		TranslateLLVMBlock (localSymbols, pair, llvmFunction, cil, module, context, globalLLVMSymbols);
	}

    // Validate the generated code, checking for consistency.
    dAssert (!llvm::verifyFunction(*llvmFunction));

    cil.Optimize (llvmFunction);

}