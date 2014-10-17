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
#include "dCILInstr.h"
#include "dDataFlowGraph.h"
#include "dCILInstrBranch.h"
#include "dCILInstrMiscellaneous.h"


dString dCIL::m_variableUndercore ("_");
dString dCIL::m_pointerDecoration ("*");
dString dCIL::m_functionArgument ("_arg");
dString dCIL::m_phiSource ("phi_source");
dString dCIL::m_pointerSize (int (sizeof (int)));

dCIL::dCIL(llvm::Module* const module)
	:dList()
	,m_mark(1)
	,m_tempIndex (0)
	,m_labelIndex (0)
    ,m_optimizer(module)
{
	memset (m_conditionals, 0, sizeof (m_conditionals));
	m_conditionals[dCILThreeArgInstr::m_identical] = dCILThreeArgInstr::m_identical;
	m_conditionals[dCILThreeArgInstr::m_different] = dCILThreeArgInstr::m_different;
	m_conditionals[dCILThreeArgInstr::m_less] = dCILThreeArgInstr::m_less;
	m_conditionals[dCILThreeArgInstr::m_lessEqual] = dCILThreeArgInstr::m_lessEqual;
	m_conditionals[dCILThreeArgInstr::m_greather] = dCILThreeArgInstr::m_greather;
	m_conditionals[dCILThreeArgInstr::m_greatherEqual] = dCILThreeArgInstr::m_greatherEqual;

	memset (m_operatorComplement, 0, sizeof (m_operatorComplement));
	m_operatorComplement[dCILThreeArgInstr::m_identical] = dCILThreeArgInstr::m_different;
	m_operatorComplement[dCILThreeArgInstr::m_different] = dCILThreeArgInstr::m_identical;
	m_operatorComplement[dCILThreeArgInstr::m_less] = dCILThreeArgInstr::m_greatherEqual;
	m_operatorComplement[dCILThreeArgInstr::m_lessEqual] = dCILThreeArgInstr::m_greather;
	m_operatorComplement[dCILThreeArgInstr::m_greather] = dCILThreeArgInstr::m_lessEqual;
	m_operatorComplement[dCILThreeArgInstr::m_greatherEqual] = dCILThreeArgInstr::m_less;

	memset (m_commutativeOperator, false, sizeof (m_commutativeOperator));
	m_commutativeOperator[dCILThreeArgInstr::m_add] = true;
	m_commutativeOperator[dCILThreeArgInstr::m_mul] = true;
	m_commutativeOperator[dCILThreeArgInstr::m_identical] = true;
	m_commutativeOperator[dCILThreeArgInstr::m_different] = true;


    // Promote allocas to registers.
    m_optimizer.add(llvm::createPromoteMemoryToRegisterPass());
	m_optimizer.add(llvm::createReassociatePass());
/*
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
*/
    m_optimizer.doInitialization();
}

dCIL::~dCIL(void)
{
	Clear();
}


void dCIL::Clear()
{
	ResetTemporaries();
	while (GetLast()) {
		delete GetLast()->GetInfo();
	}
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
		node->GetInfo()->Trace();
	}
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

dCILInstr::dArgType dCIL::GetType (const llvm::Type* const type) const
{
	const llvm::Type* myType = type;
	dCILInstr::dArgType intrinsicType (dCILInstr::m_int);
	llvm::Type::TypeID typeId = myType->getTypeID();

	if (typeId == llvm::Type::TypeID::PointerTyID) {
		intrinsicType.m_isPointer = true;
		myType = myType->getPointerElementType();
		dAssert (myType);
		typeId = myType->getTypeID();
	}

	switch (typeId)
	{
		case llvm::Type::TypeID::VoidTyID:
		{
			intrinsicType.m_intrinsicType = dCILInstr::m_void;
			break;
		}

		case llvm::Type::TypeID::IntegerTyID:
		{
//			unsigned xxx = myType->getSubclassData();
			const llvm::Type* const xxx = myType->getScalarType() ;

			if (myType->isIntegerTy (32)) {
				intrinsicType.m_intrinsicType = dCILInstr::m_int;
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


dCILInstr::dArgType dCIL::GetType (const llvm::Value* const value) const
{
	dCILInstr::dArgType intrinsicType (dCILInstr::m_int);
	llvm::Value::ValueTy valueId = llvm::Value::ValueTy (value->getValueID());
	switch (valueId)
	{
		case llvm::Value::ConstantIntVal:
		{
			intrinsicType.m_intrinsicType = dCILInstr::m_constInt;
			break;
		}

		case llvm::Value::ConstantFPVal:
		{
			dAssert (0);
			break;
		}

		default:
		{
			intrinsicType = GetType(value->getType());
		}
	}
	return intrinsicType;
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
dAssert (0);
return NULL;
/*
	const llvm::StringRef& functionName = llvmFunction.getName();
	const llvm::Type* const returnType = llvmFunction.getReturnType();
	llvm::Type::TypeID returnTypeID = returnType->getTypeID();

	dCIL::dListNode* const functionNode = NewStatement();
	dCILInstr& function = functionNode->GetInfo();
	function.m_instruction = dCILInstr::m_function;
	function.m_arg0.m_label = functionName.data();
	function.m_arg0.SetType (GetType (returnType));
	DTRACE_INTRUCTION (&function);

	dCIL::dListNode* const entryPointNode = NewStatement();
	dCILInstr& entryPoint = entryPointNode->GetInfo();
	entryPoint.m_instruction = dCILInstr::m_label;
	entryPoint.m_arg0.m_label = "entryPoint";
	DTRACE_INTRUCTION (&entryPoint);

	bool isFirst = true;
	for (llvm::Function::const_arg_iterator iter (llvmFunction.arg_begin()); iter != llvmFunction.arg_end(); iter ++) {
		const llvm::Argument* const argument = iter;
		const llvm::StringRef& name = argument->getName();

		const llvm::Type* const argType = argument->getType();
		dCIL::dListNode* const argNode = NewStatement();
		dCILInstr& stmt = argNode->GetInfo();

		stmt.m_instruction = dCILInstr::m_argument;
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
		dCILInstr& stmt = argNode->GetInfo();

		stmt.m_instruction = dCILInstr::m_assigment;
		stmt.m_operator = dCILInstr::m_nothing;
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
*/
}

void dCIL::EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dTree<dCIL::dListNode*, const llvm::BasicBlock*>& visited, dTree<dCIL::dListNode*, const llvm::BasicBlock*>& terminalInstructions)
{
	if (!visited.Find (block)) {
		dCIL::dListNode* const blockNode = EmitBasicBlockBody (function, block, terminalInstructions);
		visited.Insert (blockNode, block);

		const llvm::TerminatorInst* const teminatorIntruction = block->getTerminator();
		int successorsCount = teminatorIntruction->getNumSuccessors();
		for (int i = 0; i < successorsCount; i ++) { 
		//for (int i = successorsCount - 1; i >= 0; i --) { 
			const llvm::BasicBlock* const successorBlock = teminatorIntruction->getSuccessor(i);
			EmitBasicBlockBody (function, successorBlock, visited, terminalInstructions);
		}
	}
}

void dCIL::ConvertLLVMFunctionToNVMFunction (const llvm::Function& llvmFunction)
{
dAssert (0);
/*
	// emet function decalaration
	dCIL::dListNode* const function = EmitFunctionDeclaration (llvmFunction);

	// iterate over bascia block and emit the block body
	dTree<dCIL::dListNode*, const llvm::BasicBlock*> visited;
	dTree<dCIL::dListNode*, const llvm::BasicBlock*> terminalInstructions;
	const llvm::BasicBlock* const entryBlock = &llvmFunction.getEntryBlock();
	EmitBasicBlockBody (llvmFunction, entryBlock, visited, terminalInstructions);

//Trace();
	// bind Basic blocks 
	dTree<dCIL::dListNode*, const llvm::BasicBlock*>::Iterator iter(terminalInstructions);
	for (iter.Begin(); iter; iter ++) {
		dCIL::dListNode* const node = iter.GetNode()->GetInfo();
		dCILInstr& stmt = node->GetInfo();

DTRACE_INTRUCTION (&stmt);
		switch (stmt.m_instruction)
		{
			case dCILInstr::m_if:
			{
				llvm::BasicBlock* const trueTargetJump = (llvm::BasicBlock*) stmt.m_trueTargetJump;
				llvm::BasicBlock* const falseTargetJump = (llvm::BasicBlock*) stmt.m_falseTargetJump;
				stmt.m_trueTargetJump = (dCIL::dListNode*) visited.Find (trueTargetJump)->GetInfo();
				stmt.m_falseTargetJump = (dCIL::dListNode*) visited.Find (falseTargetJump)->GetInfo();

				dCIL::dListNode* ptr = node->GetNext();
				for (; ptr && ptr->GetInfo().m_instruction != dCILInstr::m_label; ptr = ptr->GetNext());
				if (stmt.m_falseTargetJump != ptr) {
					dAssert (stmt.m_trueTargetJump == ptr);
					dSwap (stmt.m_falseTargetJump, stmt.m_trueTargetJump);
					dSwap (stmt.m_arg1, stmt.m_arg2);
					stmt.m_instruction = dCILInstr::m_ifnot;
					DTRACE_INTRUCTION (&stmt);
				}

				break;
			}

			case dCILInstr::m_goto:
			{
				llvm::BasicBlock* const trueTargetJump = (llvm::BasicBlock*) stmt.m_trueTargetJump;
				stmt.m_trueTargetJump = (dCIL::dListNode*) visited.Find (trueTargetJump)->GetInfo();
				break;
			}

			case dCILInstr::m_ret:
				break;

			default:
				dAssert (0);
				break;
		}
	}


	// resolve PhiNodes
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = function; node; node = nextNode) {
		nextNode = node->GetNext();
		dCILInstr& stmt = node->GetInfo();
//DTRACE_INTRUCTION (&stmt);
		if (stmt.m_instruction == dCILInstr::m_phi) {
			dCIL::dListNode* nextNode1;
			for (dCIL::dListNode* node1 = node->GetPrev(); node1 && (node1->GetInfo().m_instruction == dCILInstr::m_nop); node1 = nextNode1) {
				nextNode1 = node1->GetPrev();
				dCILInstr& variableStmt = node1->GetInfo();
				dAssert (variableStmt.m_arg0.m_label == m_phiSource);
				llvm::BasicBlock* const sourceBlock = (llvm::BasicBlock*) variableStmt.m_trueTargetJump;
				dAssert (sourceBlock);
				dAssert (terminalInstructions.Find (sourceBlock));
				dCIL::dListNode* const terminalNode = (dCIL::dListNode*) terminalInstructions.Find (sourceBlock)->GetInfo();
				dAssert (terminalNode);
				dCIL::dListNode* const assigmentNode = NewStatement();
				InsertAfter (terminalNode->GetPrev(), assigmentNode);

				dCILInstr& assigmentStmt = assigmentNode->GetInfo();
				assigmentStmt.m_instruction = dCILInstr::m_assigment;
				assigmentStmt.m_operator = dCILInstr::m_nothing;
				assigmentStmt.m_arg0 = stmt.m_arg0;
				assigmentStmt.m_arg1 = variableStmt.m_arg1;
				dAssert (assigmentStmt.m_arg0.m_intrinsicType == assigmentStmt.m_arg1.m_intrinsicType);
//DTRACE_INTRUCTION (&terminalNode->GetInfo());
				Remove (node1);
			}
			Remove (node);
		}
	}

//Trace();
	RegisterAllocation (function);
*/
}


dCIL::dListNode* dCIL::EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dTree<dCIL::dListNode*, const llvm::BasicBlock*>& terminalInstructions)
{
dAssert (0);
return NULL;
/*
	const llvm::StringRef& blockName = block->getName ();

	dCIL::dListNode* const blockLabelNode = NewStatement();
	dCILInstr& blockLabel = blockLabelNode->GetInfo();
	blockLabel.m_instruction = dCILInstr::m_label;
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
			case llvm::Instruction::SDiv:
			{
				node = EmitIntegerAritmetic (intruction);
				break;
			}

			case llvm::Instruction::PHI:
			{
				node = EmitPhiNode (intruction);
				break;
			}
			
			case llvm::Instruction::GetElementPtr:
			{
				node = EmitGetElementPtr (intruction);
				break;
			}

			case llvm::Instruction::Load:
			{
				node = EmitLoad (intruction);
				break;
			}

			case llvm::Instruction::Store:
			{
				node = EmitStore (intruction);
				break;
			}

			default:
				dAssert (0);
		}

		if (intruction->isTerminator()) {
			terminalInstructions.Insert (node, block);
		}
	}

	return blockLabelNode;
*/
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
		dCILInstr& stmt = node->GetInfo();
		stmt.m_instruction = dCILInstr::m_param;
		stmt.m_arg0.SetType (GetType (arg));
		stmt.m_arg0.m_label = GetName (arg);
		DTRACE_INTRUCTION (&stmt);
	}

	dCIL::dListNode* node = NewStatement();
	dCILInstr& stmt = node->GetInfo();

	llvm::Value* const arg = instr->getOperand(argCount - 1);
	stmt.m_instruction = dCILInstr::m_call;

	stmt.m_arg0.SetType (GetType (instr));
	stmt.m_arg0.m_label = GetReturnVariableName();

	stmt.m_arg1.SetType (stmt.m_arg0.GetType());
	stmt.m_arg1.m_label = GetName (arg);
	DTRACE_INTRUCTION (&stmt);

	if ((stmt.m_arg0.m_isPointer) || (stmt.m_arg0.m_intrinsicType != dCILInstr::m_void)) { 
		dCIL::dListNode* const copyNode = NewStatement();
		dCILInstr& copyStmt = copyNode->GetInfo();

		copyStmt.m_instruction = dCILInstr::m_assigment;
		copyStmt.m_operator = dCILInstr::m_nothing;
		copyStmt.m_arg0.SetType(stmt.m_arg0.GetType());
		copyStmt.m_arg0.m_label = instr->getName().data();

		copyStmt.m_arg1.SetType(stmt.m_arg0.GetType());
		copyStmt.m_arg1.m_label = stmt.m_arg0.m_label;
		DTRACE_INTRUCTION (&copyStmt);
		node = copyNode;
	}
	return node;
*/
}

dCIL::dListNode* dCIL::EmitReturn (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::ReturnInst* const instr =  (llvm::ReturnInst*) intruction;


	dCILInstr::dArgType type (dCILInstr::m_void);
	dString arg1Label ("");
	if (instr->getNumOperands() == 1) {
		llvm::Value* const arg0 = instr->getOperand(0);

		arg1Label = GetName (arg0);
		type = GetType (arg0);
		switch (type.m_intrinsicType) 
		{
			case dCILInstr::m_constInt:
			{
				dAssert (0);
#if 0
				dCIL::dListNode* const node = NewStatement();
				dCILInstr& stmt = node->GetInfo();

				stmt.m_instruction = dCILInstr::m_assigment;
				stmt.m_operator = dCILInstr::m_nothing;

				stmt.m_arg0.m_type = dCILInstr::m_int;
				stmt.m_arg0.m_label = GetReturnVariableName();

				stmt.m_arg1.m_type = type;
				stmt.m_arg1.m_label = arg1Label;

				type = dCILInstr::m_int;
				arg1Label = stmt.m_arg0.m_label;

				DTRACE_INTRUCTION (&stmt);
#endif
				break;
			}

			case dCILInstr::m_int:
				dAssert (0);
				break;

			default:
				dAssert (0);
		}
	}

	dCIL::dListNode* const node = NewStatement();
	dCILInstr& stmt = node->GetInfo();

	stmt.m_instruction = dCILInstr::m_ret;
	stmt.m_arg0.SetType(type);
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
	dCILInstr::dOperator predicateOperator = dCILInstr::m_identical;
	switch (predicate)
	{
		case llvm::CmpInst::ICMP_EQ:
		{
			predicateOperator = dCILInstr::m_identical;
			break;
		}

		case llvm::CmpInst::ICMP_SLE:
		{
			predicateOperator = dCILInstr::m_lessEqual;
			break;
		}

		case llvm::CmpInst::ICMP_SGE:
		{
			predicateOperator = dCILInstr::m_greatherEqual;
			break;
		}

		case llvm::CmpInst::ICMP_SLT:
		{
			predicateOperator = dCILInstr::m_less;
			break;
		}
		
		case llvm::CmpInst::ICMP_SGT:
		{
			predicateOperator = dCILInstr::m_greather;
			break;
		}

		default:
			dAssert (0);
			break;
	}

	dAssert (instr->getNumOperands() == 2);
	llvm::Value* const arg0 = instr->getOperand(0);
	llvm::Value* const arg1 = instr->getOperand(1);

	dCIL::dListNode* const node = NewStatement();
	dCILInstr& stmt = node->GetInfo();
	stmt.m_instruction = dCILInstr::m_assigment;

	stmt.m_operator = predicateOperator;

	stmt.m_arg0.SetType (dCILInstr::m_int);
	stmt.m_arg0.m_label = instr->getName().data();

	stmt.m_arg1.SetType (dCILInstr::m_int);
	stmt.m_arg1.m_label = GetName (arg0);

	stmt.m_arg2.SetType (GetType (arg1));
	stmt.m_arg2.m_label = GetName (arg1);
	DTRACE_INTRUCTION (&stmt);
	return node;
*/
}

dCIL::dListNode* dCIL::EmitIntegerBranch (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*

	llvm::BranchInst* const instr =  (llvm::BranchInst*) intruction;

	dCIL::dListNode* const node = NewStatement();
	dCILInstr& stmt = node->GetInfo();
	llvm::Value* const arg0 = instr->getOperand(0);
	if (instr->getNumOperands() == 3) {
		llvm::Value* const arg2 = instr->getOperand(1);
		llvm::Value* const arg1 = instr->getOperand(2);

		//llvm::CmpInst::Predicate predicate = ((llvm::ICmpInst*) arg0)->getPredicate();
		//dAssert (((llvm::ICmpInst*) arg0)->getPredicate() == llvm::ICmpInst::ICMP_EQ);

		 dAssert (instr->getNumSuccessors() == 2);
		 const llvm::BasicBlock* const block1 = instr->getSuccessor(0);
		 const llvm::BasicBlock* const block2 = instr->getSuccessor(1);

		stmt.m_instruction = dCILInstr::m_if;
		stmt.m_arg0.SetType (dCILInstr::m_int);
		stmt.m_arg0.m_label = arg0->getName().data();
		stmt.m_arg1.m_label = arg1->getName().data();
		stmt.m_arg2.m_label = arg2->getName().data();

		// use thsi to save the link later
		stmt.m_trueTargetJump = (dList<dCILInstr>::dListNode*) arg1;
		stmt.m_falseTargetJump = (dList<dCILInstr>::dListNode*) arg2;
	} else {
		dAssert (instr->getNumOperands() == 1);
		stmt.m_instruction = dCILInstr::m_goto;
		stmt.m_operator = dCILInstr::m_nothing;
		stmt.m_arg0.m_label = arg0->getName().data();
		// use thsi to save the link later
		stmt.m_trueTargetJump = (dList<dCILInstr>::dListNode*) arg0;
	}

	DTRACE_INTRUCTION (&stmt);
	return node;
*/
}

dCIL::dListNode* dCIL::EmitPhiNode (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::PHINode* const instr =  (llvm::PHINode*) intruction;
	int agrCount = instr->getNumIncomingValues();

	for (int i = 0; i < agrCount; i ++) {
		llvm::Value* const variable = instr->getIncomingValue(i);
		llvm::BasicBlock* const block = instr->getIncomingBlock (i);
		const llvm::StringRef& variableName = variable->getName ();

		dCIL::dListNode* const node = NewStatement();
		dCILInstr& stmt = node->GetInfo();
		stmt.m_instruction = dCILInstr::m_nop;
		stmt.m_arg0.m_label = m_phiSource;
		stmt.m_arg1.m_label = variable->getName().data();
		stmt.m_arg2.m_label = block->getName().data();
		stmt.m_trueTargetJump = (dCIL::dListNode*)block;
		DTRACE_INTRUCTION (&stmt);
	}

	llvm::Type* const type = instr->getType();
	const llvm::StringRef& name = instr->getName ();

	dCIL::dListNode* const node = NewStatement();
	dCILInstr& stmt = node->GetInfo();
	stmt.m_instruction = dCILInstr::m_phi;
	stmt.m_arg0.SetType (GetType (type));
	stmt.m_arg0.m_label = name.data();
	DTRACE_INTRUCTION (&stmt);

	return  node;
*/
}

dCIL::dListNode* dCIL::EmitLoad (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::LoadInst* const instr =  (llvm::LoadInst*) intruction;

	dAssert (instr->getNumOperands() == 1);
	llvm::Value* const arg0 = instr->getOperand(0);

	dCILInstr::dArgType type (GetType(instr->getType()));

	dCIL::dListNode* const node = NewStatement();
	dCILInstr& stmt = node->GetInfo();
	stmt.m_instruction = dCILInstr::m_loadBase;
	stmt.m_operator = dCILInstr::m_nothing;
	stmt.m_arg0.SetType(type);
	stmt.m_arg0.m_label = instr->getName().data();

	stmt.m_arg1.SetType(GetType (arg0));
	stmt.m_arg1.m_label = GetName (arg0);

	DTRACE_INTRUCTION (&stmt);
	return node;
*/
}

dCIL::dListNode* dCIL::EmitStore (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*

	llvm::StoreInst* const instr =  (llvm::StoreInst*) intruction;

	dAssert (instr->getNumOperands() == 2);
	llvm::Value* const arg0 = instr->getOperand(1);
	llvm::Value* const arg1 = instr->getOperand(0);

	dCILInstr::dArgType type (GetType(instr->getType()));

	dCIL::dListNode* const node = NewStatement();
	dCILInstr& stmt = node->GetInfo();
	stmt.m_instruction = dCILInstr::m_storeBase;
	stmt.m_operator = dCILInstr::m_nothing;

	stmt.m_arg0.SetType(GetType (arg0));
	stmt.m_arg0.m_label = GetName (arg0);

	stmt.m_arg1.SetType(GetType (arg1));
	stmt.m_arg1.m_label = GetName (arg1);

	DTRACE_INTRUCTION (&stmt);
	return node;
*/
}


dCIL::dListNode* dCIL::EmitGetElementPtr (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::GetElementPtrInst* const instr =  (llvm::GetElementPtrInst*) intruction;

	int agrCount = instr->getNumOperands();

	dAssert (instr->getNumOperands() == 2);
	llvm::Value* const arg0 = instr->getOperand(0);
	llvm::Value* const arg1 = instr->getOperand(1);

	dCILInstr::dArgType type (GetType(instr->getType()));

	dCIL::dListNode* const node1 = NewStatement();
	dCILInstr& stmt1 = node1->GetInfo();
	stmt1.m_instruction = dCILInstr::m_assigment;
	stmt1.m_operator = dCILInstr::m_mul;
	stmt1.m_arg0.SetType(GetType (arg1));
	stmt1.m_arg0.m_label = GetName (arg1);
	stmt1.m_arg1 = stmt1.m_arg0;
	stmt1.m_arg2.SetType(dCILInstr::m_constInt);
	stmt1.m_arg2.m_label = dString (GetType(arg0).GetSizeInByte());
	DTRACE_INTRUCTION (&stmt1);

	dCIL::dListNode* const node = NewStatement();
	dCILInstr& stmt = node->GetInfo();
	stmt.m_instruction = dCILInstr::m_assigment;
	stmt.m_operator = dCILInstr::m_add;
	stmt.m_arg0.SetType(type);
	stmt.m_arg0.m_label = instr->getName().data();

	stmt.m_arg1.SetType(GetType (arg0));
	stmt.m_arg1.m_label = GetName (arg0);

	stmt.m_arg2.SetType(GetType (arg1));
	stmt.m_arg2.m_label = GetName (arg1);
	DTRACE_INTRUCTION (&stmt);
	return  node;
*/
}


dCIL::dListNode* dCIL::EmitIntegerAritmetic (const llvm::Instruction* const intruction)
{
dAssert (0);
return NULL;
/*
	llvm::BinaryOperator* const instr = (llvm::BinaryOperator*) intruction;

	dCILInstr::dOperator operation = dCILInstr::m_nothing;
	llvm::BinaryOperator::BinaryOps opcode = instr->getOpcode();
	switch (opcode) 
	{
		case llvm::Instruction::Add:
		{
			operation = dCILInstr::m_add;
			break;
		}

		case llvm::Instruction::Sub:
		{
			operation = dCILInstr::m_sub;
			break;
		}

		case llvm::Instruction::SDiv:
		{
			operation = dCILInstr::m_div;
			break;
		}

		default:
			dAssert (0);
	}


	dAssert (instr->getNumOperands() == 2);
	llvm::Value* const arg0 = instr->getOperand(0);
	llvm::Value* const arg1 = instr->getOperand(1);

	dCIL::dListNode* const node = NewStatement();
	dCILInstr& stmt = node->GetInfo();
	stmt.m_instruction = dCILInstr::m_assigment;
	stmt.m_operator = operation;

	stmt.m_arg0.SetType(GetType(instr->getType()));
	stmt.m_arg0.m_label = instr->getName().data();

	stmt.m_arg1.SetType(GetType (arg0));
	stmt.m_arg1.m_label = GetName (arg0);

	stmt.m_arg2.SetType(GetType (arg1));
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


dVirtualMachine* dCIL::BuilExecutable()
{
	dVirtualMachine* const program = new dVirtualMachine;

	int byteCodeOffset = 0;
	dTree <dListNode*, dString> fuctionList;
	
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		dCILInstr* const instr = node->GetInfo();
		if (instr->GetAsFunction()) {
			dCILInstrFunction* const function = instr->GetAsFunction();
			fuctionList.Insert (node, function->m_name.m_label);
		}
		instr->SetByteCodeOffset(byteCodeOffset);
		byteCodeOffset += instr->GetByteCodeSize();
	}

	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		dCILInstr* const instr = node->GetInfo();
		if (instr->GetAsCall()) {
			dCILInstrCall* const call = instr->GetAsCall();
			dTree <dListNode*, dString>::dTreeNode* const targetNode = fuctionList.Find(call->GetArg1().m_label);
			if (targetNode) {
				call->SetTarget(targetNode->GetInfo());
			}
		}
	}

	dTree <dListNode*, dString>::Iterator iter (fuctionList);
	for (iter.Begin(); iter; iter ++) {
		dListNode* const node = iter.GetNode()->GetInfo();
		dCILInstrFunction* const function = node->GetInfo()->GetAsFunction();

		dVirtualMachine::dFunctionDescription::dReturnType type = dVirtualMachine::dFunctionDescription::m_void;
		if (!function->m_name.m_isPointer) {
			switch (function->m_name.m_intrinsicType)
			{
				case dCILInstr::m_int:
					type = dVirtualMachine::dFunctionDescription::m_intReg;
					break;

				case dCILInstr::m_void:
					break;
				default: 
					dAssert (0);
			}
		} else {
			type = dVirtualMachine::dFunctionDescription::m_intReg;
		}
		program->AddFunction (function->m_name.m_label, function->GetByteCodeOffset(), type);
	}


	dVirtualMachine::dOpCode* const byteCode = program->AllocCodeSegement(byteCodeOffset);

Trace();
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		dCILInstr* const instr = node->GetInfo();
instr->Trace();
		instr->EmitOpcode(byteCode);
	}

	program->ExecuteFunction ("_Fibonacci::_Fibonacci::int", "::int" , 2);


	return program;
}