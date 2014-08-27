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

#include "dCILstdafx.h"
#include "dCIL.h"
#include "dDataFlowGraph.h"

dString dCIL::m_variableUndercore ("_");
dString dCIL::m_pointerDecoration ("*");
dString dCIL::m_functionArgument ("_arg");
dString dCIL::m_pointerSize (int (sizeof (int)));

dCIL::dCIL(llvm::Module* const module)
	:dList()
	,m_mark(1)
	,m_tempIndex (0)
	,m_labelIndex (0)
    ,m_optimizer(module)
{
	memset (m_conditionals, 0, sizeof (m_conditionals));
	m_conditionals[dThreeAdressStmt::m_identical] = dThreeAdressStmt::m_identical;
	m_conditionals[dThreeAdressStmt::m_different] = dThreeAdressStmt::m_different;
	m_conditionals[dThreeAdressStmt::m_less] = dThreeAdressStmt::m_less;
	m_conditionals[dThreeAdressStmt::m_lessEqual] = dThreeAdressStmt::m_lessEqual;
	m_conditionals[dThreeAdressStmt::m_greather] = dThreeAdressStmt::m_greather;
	m_conditionals[dThreeAdressStmt::m_greatherEqual] = dThreeAdressStmt::m_greatherEqual;

	memset (m_operatorComplement, 0, sizeof (m_operatorComplement));
	m_operatorComplement[dThreeAdressStmt::m_identical] = dThreeAdressStmt::m_different;
	m_operatorComplement[dThreeAdressStmt::m_different] = dThreeAdressStmt::m_identical;
	m_operatorComplement[dThreeAdressStmt::m_less] = dThreeAdressStmt::m_greatherEqual;
	m_operatorComplement[dThreeAdressStmt::m_lessEqual] = dThreeAdressStmt::m_greather;
	m_operatorComplement[dThreeAdressStmt::m_greather] = dThreeAdressStmt::m_lessEqual;
	m_operatorComplement[dThreeAdressStmt::m_greatherEqual] = dThreeAdressStmt::m_less;

	memset (m_commutativeOperator, false, sizeof (m_commutativeOperator));
	m_commutativeOperator[dThreeAdressStmt::m_add] = true;
	m_commutativeOperator[dThreeAdressStmt::m_mul] = true;
	m_commutativeOperator[dThreeAdressStmt::m_identical] = true;
	m_commutativeOperator[dThreeAdressStmt::m_different] = true;


    // Promote allocas to registers.
    m_optimizer.add(llvm::createPromoteMemoryToRegisterPass());
	m_optimizer.add(llvm::createReassociatePass());

	m_optimizer.add(llvm::createDeadInstEliminationPass());
	m_optimizer.add(llvm::createDeadCodeEliminationPass());
	m_optimizer.add(llvm::createConstantHoistingPass());
	m_optimizer.add(llvm::createConstantPropagationPass());
	m_optimizer.add(llvm::createInstructionCombiningPass());
	m_optimizer.add(llvm::createMergedLoadStoreMotionPass());

	m_optimizer.add(llvm::createGVNPass ());
	m_optimizer.add(llvm::createCFGSimplificationPass());
	m_optimizer.add(llvm::createTailCallEliminationPass());
	m_optimizer.add(llvm::createJumpThreadingPass());

	//m_optimizer.add(llvm::createGVNPass());
	//m_optimizer.add(llvm::createCFGSimplificationPass());
	//m_optimizer.add(llvm::createLowerSwitchPass());
	
	m_optimizer.add(llvm::createLCSSAPass());
	m_optimizer.add(llvm::createLICMPass());
	m_optimizer.add(llvm::createIndVarSimplifyPass());
	m_optimizer.add(llvm::createLoopIdiomPass());
	m_optimizer.add(llvm::createLoopStrengthReducePass());
	m_optimizer.add(llvm::createLoopInstSimplifyPass());
	m_optimizer.add(llvm::createLoopUnswitchPass());

    m_optimizer.doInitialization();
}

dCIL::~dCIL(void)
{
}


void dCIL::Clear()
{
	ResetTemporaries();
	RemoveAll();
}


void dCIL::ResetTemporaries()
{
	m_tempIndex = 0;
	m_labelIndex = 0;
}

dString dCIL::NewTemp ()
{
	m_tempIndex ++;
	return GetTemporaryVariableName(m_tempIndex - 1);
}

dString dCIL::NewLabel ()
{
	char tmp[256];
	sprintf (tmp, "label_%d", m_labelIndex);
	m_labelIndex ++;
	return dString (tmp);
}

dCIL::dListNode* dCIL::NewStatement()
{
	return Append();
}


void dCIL::Trace()
{
	for (dCIL::dListNode* node = GetFirst(); node; node = node->GetNext()) {
		const dThreeAdressStmt& stmt = node->GetInfo();
//		DTRACE_INTRUCTION(&stmt);
		stmt.Trace();
	}
	dTrace(("\n"));
}



/*
//void dCIL::Optimize(dListNode* const functionNode, int argumentInRegisters, dReturnType returnType)
void dCIL::Optimize(dListNode* const functionNode, int argumentInRegisters)
{
	dAssert (0);

	// remove all redundant newly generate extra jumps 
	//RemoveRedundantJumps(functionNode);

	// create float control for inteBlock optimization
	dDataFlowGraph datFlowGraph (this, functionNode, returnType);

	// apply all basic blocks peephole optimizations 
	datFlowGraph.ApplyLocalOptimizations();

	// do register allocation before removing dead jumps and nops
	datFlowGraph.RegistersAllocation (D_INTEGER_REGISTER_COUNT - 1);

	for (bool isDirty = true; isDirty; ) {
		isDirty = false;
		// remove all redundant newly generate extra jumps 
		isDirty |= RemoveRedundantJumps(functionNode);
//Trace();

		// clean up all nop instruction added by the optimizer
		isDirty |= RemoveNop(functionNode);
//Trace();
	}

//	Trace();

}
*/



void dCIL::Optimize (llvm::Function* const function)
{
    // Optimize the function.
	bool unOpmized = true;
	for (int i = 0; (i < 32) && unOpmized; i ++) {
		unOpmized = m_optimizer.run(*function);
	}
}

dThreeAdressStmt::dArgType dCIL::GetType (const llvm::Type* const type) const
{
	const llvm::Type* myType = type;
	dThreeAdressStmt::dArgType intrinsicType (dThreeAdressStmt::m_int);
	llvm::Type::TypeID typeId = myType->getTypeID();

	if (typeId == llvm::Type::TypeID::PointerTyID) {
		intrinsicType.m_isPointer = true;
		myType = type->getPointerElementType();
		dAssert (myType);
		typeId = myType->getTypeID();
	}

	switch (typeId)
	{
		case llvm::Type::TypeID::VoidTyID:
		{
			intrinsicType.m_intrinsicType = dThreeAdressStmt::m_void;
			break;
		}

		case llvm::Type::TypeID::IntegerTyID:
		{
			if (myType->isIntegerTy (32)) {
				intrinsicType.m_intrinsicType = dThreeAdressStmt::m_int;
			} else {
				dAssert (0);
			}
			break;
		}

		default:
		{
			dAssert (0);
			break;
		}
	}


	//VoidTyID = 0,    ///<  0: type with no size
	//HalfTyID,        ///<  1: 16-bit floating point type
	//FloatTyID,       ///<  2: 32-bit floating point type
	//DoubleTyID,      ///<  3: 64-bit floating point type
	//X86_FP80TyID,    ///<  4: 80-bit floating point type (X87)
	//FP128TyID,       ///<  5: 128-bit floating point type (112-bit mantissa)
	//PPC_FP128TyID,   ///<  6: 128-bit floating point type (two 64-bits, PowerPC)
	//LabelTyID,       ///<  7: Labels
	//MetadataTyID,    ///<  8: Metadata

	return intrinsicType;
}



dThreeAdressStmt::dArgType dCIL::GetType (const llvm::Value* const value) const
{
dAssert (0);
return dThreeAdressStmt::dArgType();
/*
	dThreeAdressStmt::dArgType intrinsicType (dThreeAdressStmt::m_int);
	llvm::Value::ValueTy valueId = llvm::Value::ValueTy (value->getValueID());
	switch (valueId)
	{
		case llvm::Value::ConstantIntVal:
		{
			intrinsicType = dThreeAdressStmt::m_constInt;
			break;
		}

		case llvm::Value::ConstantFPVal:
		{
			dAssert (0);
			break;
		}

		default:
		{
			intrinsicType = GetType (value->getType());
		}
	}
	return intrinsicType;
*/
}

dString dCIL::GetName (llvm::Value* const value) const
{
	int type = value->getValueID();
	if (type == llvm::Value::ConstantIntVal) {
		llvm::ConstantInt* constValue = (llvm::ConstantInt*) value;
		return dString (int (constValue->getZExtValue()));
	} else {
		return value->getName().data();
	}
}

dCIL::dListNode* dCIL::EmitFunctionDeclaration (const llvm::Function& llvmFunction)
{
	const llvm::StringRef& functionName = llvmFunction.getName();
	const llvm::Type* const returnType = llvmFunction.getReturnType();
	llvm::Type::TypeID returnTypeID = returnType->getTypeID();

	dCIL::dListNode* const functionNode = NewStatement();
	dThreeAdressStmt& function = functionNode->GetInfo();
	function.m_instruction = dThreeAdressStmt::m_function;
	function.m_arg0.m_label = functionName.data();
	function.m_arg0.SetType (GetType (returnType));
	DTRACE_INTRUCTION (&function);

	dCIL::dListNode* const entryPointNode = NewStatement();
	dThreeAdressStmt& entryPoint = entryPointNode->GetInfo();
	entryPoint.m_instruction = dThreeAdressStmt::m_label;
	entryPoint.m_arg0.m_label = "entryPoint";
	DTRACE_INTRUCTION (&entryPoint);

	bool isFirst = true;
	for (llvm::Function::const_arg_iterator iter (llvmFunction.arg_begin()); iter != llvmFunction.arg_end(); iter ++) {
		const llvm::Argument* const argument = iter;
		const llvm::StringRef& name = argument->getName();

		const llvm::Type* const argType = argument->getType();
		dCIL::dListNode* const argNode = NewStatement();
		dThreeAdressStmt& stmt = argNode->GetInfo();

		stmt.m_instruction = dThreeAdressStmt::m_argument;
		stmt.m_arg0.SetType (GetType (argType));
		if (isFirst && (returnTypeID != llvm::Type::TypeID::VoidTyID)) {
			stmt.m_arg0.m_label = GetReturnVariableName();		
		} else {
			stmt.m_arg0.m_label = dString(name.data()) + m_functionArgument;
		}
		isFirst = false;
		DTRACE_INTRUCTION (&stmt);
	}

	isFirst = true;
	for (llvm::Function::const_arg_iterator iter (llvmFunction.arg_begin()); iter != llvmFunction.arg_end(); iter ++) {
		const llvm::Argument* const argument = iter;
		const llvm::StringRef& name = argument->getName();

		const llvm::Type* const argType = argument->getType();

		dCIL::dListNode* const argNode = NewStatement();
		dThreeAdressStmt& stmt = argNode->GetInfo();

		stmt.m_instruction = dThreeAdressStmt::m_assigment;
		stmt.m_operator = dThreeAdressStmt::m_nothing;
		stmt.m_arg0.SetType (GetType (argType));
		stmt.m_arg0.m_label = name.data();

		stmt.m_arg1.SetType (stmt.m_arg0.GetType());
		if (isFirst && (returnTypeID != llvm::Type::TypeID::VoidTyID)) {
			stmt.m_arg1.m_label = GetReturnVariableName();
		} else {
			stmt.m_arg1.m_label = stmt.m_arg0.m_label + m_functionArgument;
		}
		isFirst = false;
		DTRACE_INTRUCTION (&stmt);
	}
	return functionNode;
}

void dCIL::EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dTree<const dCIL::dListNode*, const llvm::BasicBlock*>& visited, dList<dCIL::dListNode*>& terminalInstructions)
{
	if (!visited.Find (block)) {
		const dCIL::dListNode* const blockNode = EmitBasicBlockBody (function, block, terminalInstructions);
		visited.Insert (blockNode, block);

		const llvm::TerminatorInst* const teminatorIntruction = block->getTerminator();
		int successorsCount = teminatorIntruction->getNumSuccessors();
		//for (int i = 0; i < successorsCount; i ++) { 
		for (int i = successorsCount - 1; i >= 0; i --) { 
			const llvm::BasicBlock* const successorBlock = teminatorIntruction->getSuccessor(i);
			EmitBasicBlockBody (function, successorBlock, visited, terminalInstructions);
		}
	}
}

void dCIL::ConvertLLVMFunctionToNVMFunction (const llvm::Function& llvmFunction)
{
	// emet function decalaration
	dCIL::dListNode* const function = EmitFunctionDeclaration (llvmFunction);

	// iterate over bascia block and emit the block body
	dList<dCIL::dListNode*> terminalInstructions;
	dTree<const dCIL::dListNode*, const llvm::BasicBlock*> visited;
	const llvm::BasicBlock* const entryBlock = &llvmFunction.getEntryBlock();
	EmitBasicBlockBody (llvmFunction, entryBlock, visited, terminalInstructions);

	for (dList<dCIL::dListNode*>::dListNode* node = terminalInstructions.GetFirst(); node; node = node->GetNext()) {
		dThreeAdressStmt& stmt = node->GetInfo()->GetInfo();

//DTRACE_INTRUCTION (&stmt);
		switch (stmt.m_instruction)
		{
			case dThreeAdressStmt::m_if:
			{
				llvm::BasicBlock* const trueTargetJump = (llvm::BasicBlock*) stmt.m_trueTargetJump;
				llvm::BasicBlock* const falseTargetJump = (llvm::BasicBlock*) stmt.m_falseTargetJump;
				stmt.m_trueTargetJump = (dCIL::dListNode*) visited.Find (trueTargetJump)->GetInfo();
				stmt.m_falseTargetJump = (dCIL::dListNode*) visited.Find (falseTargetJump)->GetInfo();
				break;
			}

			case dThreeAdressStmt::m_goto:
			{
				llvm::BasicBlock* const trueTargetJump = (llvm::BasicBlock*) stmt.m_trueTargetJump;
				stmt.m_trueTargetJump = (dCIL::dListNode*) visited.Find (trueTargetJump)->GetInfo();
				break;
			}

			case dThreeAdressStmt::m_ret:
				break;

			default:
				dAssert (0);
				break;
		}
	}

	RegisterAllocation (function);
}


const dCIL::dListNode*dCIL::EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dList<dCIL::dListNode*>& terminalInstructions)
{
	const llvm::StringRef& blockName = block->getName ();

	dCIL::dListNode* const blockLabelNode = NewStatement();
	dThreeAdressStmt& blockLabel = blockLabelNode->GetInfo();
	blockLabel.m_instruction = dThreeAdressStmt::m_label;
	blockLabel.m_arg0.m_label = blockName.data();
	DTRACE_INTRUCTION (&blockLabel);

	for (llvm::BasicBlock::const_iterator iter (block->begin()); iter != block->end(); iter ++) {
		const llvm::Instruction* const intruction = iter;
		int opcode = intruction->getOpcode();

		//const char* const xxx = intruction->getOpcodeName();

		dCIL::dListNode* node = NULL;
		switch (opcode)
		{
			case llvm::Instruction::Ret:
			{
				node = EmitReturn (intruction);
				break;
			}

			case llvm::Instruction::Call:
				node = EmitCall (intruction);
				break;

			case llvm::Instruction::ICmp:
			{
				node = EmitIntegerCompare (intruction);
				break;
			}

			case llvm::Instruction::Br:
			{
				node = dCIL::EmitIntegerBranch (intruction);
				break;
			}

			case llvm::Instruction::Add:
			case llvm::Instruction::Sub:
			{
				node = EmitIntegerAritmetic (intruction);
				break;
			}

			case llvm::Instruction::PHI:
			{
				node = EmitPhiNode (intruction);
				break;
			}
			
				
			default:
				dAssert (0);
		}

		if (intruction->isTerminator()) {
			terminalInstructions.Append (node);
		}
	}

	return blockLabelNode;
}

dCIL::dListNode* dCIL::EmitCall (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::CallInst* const instr =  (llvm::CallInst*) intruction;

	int argCount = instr->getNumOperands();
	for (int i = 0; i < argCount - 1; i ++) {
		llvm::Value* const arg = instr->getOperand(i);

		dCIL::dListNode* const node = NewStatement();
		dThreeAdressStmt& stmt = node->GetInfo();
		stmt.m_instruction = dThreeAdressStmt::m_param;
		stmt.m_arg0.m_type = GetType (arg);
		stmt.m_arg0.m_label = GetName (arg);
		DTRACE_INTRUCTION (&stmt);
	}

	dCIL::dListNode* const node = NewStatement();
	dThreeAdressStmt& stmt = node->GetInfo();

	llvm::Value* const arg = instr->getOperand(argCount - 1);
	stmt.m_instruction = dThreeAdressStmt::m_call;

	stmt.m_arg0.m_type = GetType (instr);
	stmt.m_arg0.m_label = GetReturnVariableName();

	stmt.m_arg1.m_type = stmt.m_arg0.m_type;
	stmt.m_arg1.m_label = GetName (arg);
	DTRACE_INTRUCTION (&stmt);

	dCIL::dListNode* const copyNode = NewStatement();
	dThreeAdressStmt& copyStmt = copyNode->GetInfo();

	copyStmt.m_instruction = dThreeAdressStmt::m_assigment;
	copyStmt.m_operator = dThreeAdressStmt::m_nothing;
	copyStmt.m_arg0.m_type = stmt.m_arg0.m_type;
	copyStmt.m_arg0.m_label = instr->getName().data();

	copyStmt.m_arg1.m_type = stmt.m_arg0.m_type;
	copyStmt.m_arg1.m_label = stmt.m_arg0.m_label;
	DTRACE_INTRUCTION (&copyStmt);

	return copyNode;
*/
}

dCIL::dListNode* dCIL::EmitReturn (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::ReturnInst* const instr =  (llvm::ReturnInst*) intruction;

	dAssert (instr->getNumOperands() == 1);
	llvm::Value* const arg0 = instr->getOperand(0);

	dString arg1Label (GetName (arg0));
	dThreeAdressStmt::dArgType type = GetType (arg0);
	switch (type) 
	{
		case dThreeAdressStmt::m_constInt:
		{
			dCIL::dListNode* const node = NewStatement();
			dThreeAdressStmt& stmt = node->GetInfo();

			stmt.m_instruction = dThreeAdressStmt::m_assigment;
			stmt.m_operator = dThreeAdressStmt::m_nothing;

			stmt.m_arg0.m_type = dThreeAdressStmt::m_int;
			stmt.m_arg0.m_label = GetReturnVariableName();

			stmt.m_arg1.m_type = type;
			stmt.m_arg1.m_label = arg1Label;

			type = dThreeAdressStmt::m_int;
			arg1Label = stmt.m_arg0.m_label;

			DTRACE_INTRUCTION (&stmt);
			break;
		}

		case dThreeAdressStmt::m_int:
			break;

		default:
			dAssert (0);
	}

	dCIL::dListNode* const node = NewStatement();
	dThreeAdressStmt& stmt = node->GetInfo();

	stmt.m_instruction = dThreeAdressStmt::m_ret;
	stmt.m_arg0.m_type = type;
	stmt.m_arg0.m_label = arg1Label;

	DTRACE_INTRUCTION (&stmt);
	return node;
*/
}

dCIL::dListNode* dCIL::EmitIntegerCompare (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::ICmpInst* const instr =  (llvm::ICmpInst*) intruction;

	llvm::CmpInst::Predicate predicate = instr->getPredicate();
	dThreeAdressStmt::dOperator prediccateOperator = dThreeAdressStmt::m_identical;;
	switch (predicate)
	{
		case llvm::CmpInst::FCMP_OEQ:
		{
			prediccateOperator = dThreeAdressStmt::m_identical;
		}

		default:
			break;
	}

	dAssert (instr->getNumOperands() == 2);
	llvm::Value* const arg0 = instr->getOperand(0);
	llvm::Value* const arg1 = instr->getOperand(1);

	dCIL::dListNode* const node = NewStatement();
	dThreeAdressStmt& stmt = node->GetInfo();
	stmt.m_instruction = dThreeAdressStmt::m_assigment;

	stmt.m_operator = prediccateOperator;

	stmt.m_arg0.m_type = dThreeAdressStmt::m_int;
	stmt.m_arg0.m_label = instr->getName().data();

	stmt.m_arg1.m_type = dThreeAdressStmt::m_int;
	stmt.m_arg1.m_label = GetName (arg0);

	stmt.m_arg2.m_type = GetType (arg1);
	stmt.m_arg2.m_label = GetName (arg1);
	DTRACE_INTRUCTION (&stmt);
	return node;
*/
}

dCIL::dListNode* dCIL::EmitIntegerBranch (const llvm::Instruction* const intruction)
{
	llvm::BranchInst* const instr =  (llvm::BranchInst*) intruction;

	dCIL::dListNode* const node = NewStatement();
	dThreeAdressStmt& stmt = node->GetInfo();
	llvm::Value* const arg0 = instr->getOperand(0);
	if (instr->getNumOperands() == 3) {
		llvm::Value* const arg2 = instr->getOperand(1);
		llvm::Value* const arg1 = instr->getOperand(2);
		dAssert (((llvm::ICmpInst*) arg0)->getPredicate() == llvm::ICmpInst::ICMP_EQ);

		stmt.m_instruction = dThreeAdressStmt::m_if;
		stmt.m_arg0.SetType (dThreeAdressStmt::m_int);
		stmt.m_arg0.m_label = arg0->getName().data();
		stmt.m_arg1.m_label = arg1->getName().data();
		stmt.m_arg2.m_label = arg2->getName().data();

		// use thsi to save the link later
		stmt.m_trueTargetJump = (dList<dThreeAdressStmt>::dListNode*) arg1;
		stmt.m_falseTargetJump = (dList<dThreeAdressStmt>::dListNode*) arg2;
	} else {
		dAssert (instr->getNumOperands() == 1);
		stmt.m_instruction = dThreeAdressStmt::m_goto;
		stmt.m_operator = dThreeAdressStmt::m_nothing;
		stmt.m_arg0.m_label = arg0->getName().data();
		// use thsi to save the link later
		stmt.m_trueTargetJump = (dList<dThreeAdressStmt>::dListNode*) arg0;
	}

	DTRACE_INTRUCTION (&stmt);
	return node;
}

dCIL::dListNode* dCIL::EmitPhiNode (const llvm::Instruction* const intruction)
{
	dAssert (0);
	return  NULL;
}

dCIL::dListNode* dCIL::EmitIntegerAritmetic (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::BinaryOperator* const instr = (llvm::BinaryOperator*) intruction;

	dThreeAdressStmt::dOperator operation = dThreeAdressStmt::m_nothing;
	llvm::BinaryOperator::BinaryOps opcode = instr->getOpcode();
	switch (opcode) 
	{
		case llvm::Instruction::Add:
		{
			operation = dThreeAdressStmt::m_add;
			break;
		}

		case llvm::Instruction::Sub:
		{
			operation = dThreeAdressStmt::m_sub;
			break;
		}

		default:
			dAssert (0);
	}


	dAssert (instr->getNumOperands() == 2);
	llvm::Value* const arg0 = instr->getOperand(0);
	llvm::Value* const arg1 = instr->getOperand(1);

	dCIL::dListNode* const node = NewStatement();
	dThreeAdressStmt& stmt = node->GetInfo();
	stmt.m_instruction = dThreeAdressStmt::m_assigment;
	stmt.m_operator = operation;

	stmt.m_arg0.m_type = dThreeAdressStmt::m_int;
	stmt.m_arg0.m_label = instr->getName().data();

	stmt.m_arg1.m_type = GetType (arg0);
	stmt.m_arg1.m_label = GetName (arg0);

	stmt.m_arg2.m_type = GetType (arg1);
	stmt.m_arg2.m_label = GetName (arg1);
	DTRACE_INTRUCTION (&stmt);
	return node;
*/
}


void dCIL::RegisterAllocation (dListNode* const functionNode)
{
	//dDataFlowGraph datFlowGraph (this, functionNode, returnType);
	dDataFlowGraph datFlowGraph (this, functionNode);

	// apply all basic blocks peephole optimizations 
	//datFlowGraph.ApplyLocalOptimizations();

	// do register allocation before removing dead jumps and nops
	datFlowGraph.RegistersAllocation (D_INTEGER_REGISTER_COUNT - 1);


}