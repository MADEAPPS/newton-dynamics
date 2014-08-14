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
#include "dNVMAsmParser.h"
#include "dNVMAsmPrinter.h"
#include "dNVMTargetMachine.h"



//llvm::Target dCIL::m_target;

dCIL::dCIL(llvm::Module* const module)
	:dList()
	,m_mark(1)
	,m_tempIndex (0)
	,m_labelIndex (0)
    ,m_optimizer(module)
{
	memset (m_conditionals, 0, sizeof (m_conditionals));
	m_conditionals[dTreeAdressStmt::m_identical] = dTreeAdressStmt::m_identical;
	m_conditionals[dTreeAdressStmt::m_different] = dTreeAdressStmt::m_different;
	m_conditionals[dTreeAdressStmt::m_less] = dTreeAdressStmt::m_less;
	m_conditionals[dTreeAdressStmt::m_lessEqual] = dTreeAdressStmt::m_lessEqual;
	m_conditionals[dTreeAdressStmt::m_greather] = dTreeAdressStmt::m_greather;
	m_conditionals[dTreeAdressStmt::m_greatherEqual] = dTreeAdressStmt::m_greatherEqual;

	memset (m_operatorComplement, 0, sizeof (m_operatorComplement));
	m_operatorComplement[dTreeAdressStmt::m_identical] = dTreeAdressStmt::m_different;
	m_operatorComplement[dTreeAdressStmt::m_different] = dTreeAdressStmt::m_identical;
	m_operatorComplement[dTreeAdressStmt::m_less] = dTreeAdressStmt::m_greatherEqual;
	m_operatorComplement[dTreeAdressStmt::m_lessEqual] = dTreeAdressStmt::m_greather;
	m_operatorComplement[dTreeAdressStmt::m_greather] = dTreeAdressStmt::m_lessEqual;
	m_operatorComplement[dTreeAdressStmt::m_greatherEqual] = dTreeAdressStmt::m_less;

	memset (m_commutativeOperator, false, sizeof (m_commutativeOperator));
	m_commutativeOperator[dTreeAdressStmt::m_add] = true;
	m_commutativeOperator[dTreeAdressStmt::m_mul] = true;
	m_commutativeOperator[dTreeAdressStmt::m_identical] = true;
	m_commutativeOperator[dTreeAdressStmt::m_different] = true;

	//llvm::legacy::FunctionPassManager functionPassManager(m_module.get());
    // Set up the optimizer pipeline.  Start with registering info about how the
    // target lays out data structures.
    //        functionPassManager.add(new DataLayout(*TheExecutionEngine->getDataLayout()));
    // Provide basic AliasAnalysis support for GVN.
    //        functionPassManager.add(createBasicAliasAnalysisPass());

    // Promote allocas to registers.
    m_optimizer.add(llvm::createPromoteMemoryToRegisterPass());
    // Do simple "peephole" optimizations and bit-twiddling optzns.
    //        functionPassManager.add(createInstructionCombiningPass());
    // Reassociate expressions.
    //        functionPassManager.add(createReassociatePass());
    // Eliminate Common SubExpressions.
    //        functionPassManager.add(createGVNPass());
    // Simplify the control flow graph (deleting unreachable blocks, etc).
    //        functionPassManager.add(createCFGSimplificationPass());

    m_optimizer.doInitialization();

	// register the target
	//InitializeTargetInfo();
	//InitializeTarget();
	//InitializeTargetMC();
	//InitializeAsmPrinter();
	//InitializeMCAsmParser();
}

dCIL::~dCIL(void)
{
}


void dCIL::Clear()
{
	ResetTemporaries();
	RemoveAll();
}

/*
void dCIL::InitializeTargetInfo()
{
	dAssert (0);
//	llvm::RegisterTarget<llvm::Triple::UnknownArch, false> X (m_target, D_VIRTUAL_MACHINE_NAME, D_VIRTUAL_MACHINE_DESCRIPTION);
}

void dCIL::InitializeTarget()
{
	dAssert (0);
//	llvm::RegisterTargetMachine<dNVMTargetMachine> X(m_target);
}


void dCIL::InitializeTargetMC() 
{
  // Register the MC asm info.

  llvm::RegisterMCAsmInfoFn X(m_target, createSparcMCAsmInfo);

  // Register the MC codegen info.
  llvm::TargetRegistry::RegisterMCCodeGenInfo(m_target, createSparcMCCodeGenInfo);

  // Register the MC instruction info.
  llvm::TargetRegistry::RegisterMCInstrInfo(m_target, createSparcMCInstrInfo);

  // Register the MC register info.
  llvm::TargetRegistry::RegisterMCRegInfo(m_target, createSparcMCRegisterInfo);

  // Register the MC subtarget info.
  llvm::TargetRegistry::RegisterMCSubtargetInfo(m_target, createSparcMCSubtargetInfo);

  // Register the MC Code Emitter.
  llvm::TargetRegistry::RegisterMCCodeEmitter(m_target, createSparcMCCodeEmitter);

  //Register the asm backend.
  llvm::TargetRegistry::RegisterMCAsmBackend(m_target, createSparcAsmBackend);

  // Register the object streamer.
  llvm::TargetRegistry::RegisterMCObjectStreamer(m_target, createMCStreamer);

  // Register the asm streamer.
  llvm::TargetRegistry::RegisterAsmStreamer(m_target, createMCAsmStreamer);

  // Register the MCInstPrinter
  llvm::TargetRegistry::RegisterMCInstPrinter(m_target, createSparcMCInstPrinter);
}


void dCIL::InitializeAsmPrinter() 
{
	dAssert (0);
//  llvm::RegisterAsmPrinter<NVMAsmPrinter> X(m_target);
}

void dCIL::InitializeMCAsmParser() 
{
	dAssert (0);
//	llvm::RegisterMCAsmParser<NVMAsmParser> X(m_target);
}
*/

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
		const dTreeAdressStmt& stmt = node->GetInfo();
//		DTRACE_INTRUCTION(&stmt);
		stmt.Trace();
	}
	dTrace(("\n"));
}

/*
bool dCIL::RemoveRedundantJumps(dListNode* const function)
{
	bool ret = false;
	dTree<int, dListNode*> jumpMap;

	// create jump and label map;
	for (dListNode* node = function; node; node = node->GetNext()) {
		const dTreeAdressStmt& stmt = node->GetInfo();
		switch (stmt.m_instruction) 
		{
			
			case dTreeAdressStmt::m_if:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_goto:
				jumpMap.Insert(0, node);
		}
	}

	// remove redundant adjacent labels
	dTree<int, dListNode*>::Iterator iter (jumpMap);
	for (iter.Begin(); iter; iter ++) {
		dListNode* const node = iter.GetKey();
		const dTreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_label) {
			dListNode* const labelNode = node->GetNext();
			if (labelNode && (labelNode->GetInfo().m_instruction == dTreeAdressStmt::m_label)) {
				dTree<int, dListNode*>::Iterator iter1 (jumpMap);
				for (iter1.Begin(); iter1; iter1 ++) {
					dListNode* const node1 = iter1.GetKey();
					dTreeAdressStmt& stmt1 = node1->GetInfo();
					if (stmt1.m_instruction == dTreeAdressStmt::m_goto) {
						if (stmt1.m_jmpTarget == labelNode)	{
							stmt1.m_jmpTarget = node;
							stmt1.m_arg0.m_label = stmt.m_arg0.m_label;
						}
					} else if (stmt1.m_instruction == dTreeAdressStmt::m_if) { 
						if (stmt1.m_jmpTarget == labelNode)	{
							stmt1.m_jmpTarget = node;	
							stmt1.m_arg2.m_label = stmt.m_arg0.m_label;
						}
					}
				}
				ret = true;
				Remove(labelNode);
				jumpMap.Remove(labelNode);
			}
		}
	}

	// redirect double indirect goto
	for (iter.Begin(); iter; iter ++) {
		dListNode* const node = iter.GetKey();
		dTreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_goto) {
			dAssert (jumpMap.Find (stmt.m_jmpTarget));
			dListNode* const targetNode = jumpMap.Find (stmt.m_jmpTarget)->GetKey();
			dTreeAdressStmt& stmt1 = targetNode->GetInfo();
			dListNode* nextGotoNode = targetNode->GetNext();
			while (nextGotoNode->GetInfo().m_instruction == dTreeAdressStmt::m_nop) {
				nextGotoNode = nextGotoNode->GetNext();
			}
			if ((stmt1.m_instruction == dTreeAdressStmt::m_label) && (nextGotoNode->GetInfo().m_instruction == dTreeAdressStmt::m_goto)) {
				const dTreeAdressStmt& stmt2 = nextGotoNode->GetInfo();
				stmt.m_arg0.m_label = stmt2.m_arg0.m_label;
				stmt.m_jmpTarget = stmt2.m_jmpTarget;
				ret = true;
			}
		} else if (stmt.m_instruction == dTreeAdressStmt::m_if) {
			dAssert (jumpMap.Find (stmt.m_jmpTarget));
			dListNode* const targetNode = jumpMap.Find (stmt.m_jmpTarget)->GetKey();
			dTreeAdressStmt& stmt1 = targetNode->GetInfo();
			dListNode* nextGotoNode = targetNode->GetNext();
			while (nextGotoNode->GetInfo().m_instruction == dTreeAdressStmt::m_nop) {
				nextGotoNode = nextGotoNode->GetNext();
			}
			if ((stmt1.m_instruction == dTreeAdressStmt::m_label) && (nextGotoNode->GetInfo().m_instruction == dTreeAdressStmt::m_goto)) {
				const dTreeAdressStmt& stmt2 = nextGotoNode->GetInfo();
				stmt.m_arg2.m_label = stmt2.m_arg0.m_label;
				stmt.m_jmpTarget = stmt2.m_jmpTarget;
				ret = true;
			}
		}
	}


	// remove jumps over jumps
	for (iter.Begin(); iter; iter ++) {
		dListNode* const node = iter.GetKey();
		dTreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_if) {
			dListNode* const gotoNode = node->GetNext();
			dTreeAdressStmt& gotoStmt = gotoNode->GetInfo();
			if (gotoStmt.m_instruction == dTreeAdressStmt::m_goto) {
				dListNode* const target = gotoNode->GetNext();
				if (stmt.m_jmpTarget == target) {
					dTreeAdressStmt& gotoStmt = gotoNode->GetInfo();
					stmt.m_operator = m_operatorComplement[stmt.m_operator];
					stmt.m_jmpTarget = gotoStmt.m_jmpTarget;
					stmt.m_arg2.m_label = gotoStmt.m_arg0.m_label;
					Remove(gotoNode);
					jumpMap.Remove(gotoNode);
					ret = true;
				}
			}
		}
	}


	// remove goto to immediate labels
	for (iter.Begin(); iter; ) {
		dListNode* const node = iter.GetKey();
		dTreeAdressStmt& stmt = node->GetInfo();
		iter ++;
		dListNode* const nextNode = node->GetNext();
		if (((stmt.m_instruction == dTreeAdressStmt::m_if) || (stmt.m_instruction == dTreeAdressStmt::m_goto)) && (stmt.m_jmpTarget == nextNode)) {
			ret = true;
			Remove(node);
			jumpMap.Remove(node);
		}
	}


	// delete unreferenced labels
	for (iter.Begin(); iter; ) {
		dListNode* const node = iter.GetKey();
		dTreeAdressStmt& stmt = node->GetInfo();
		iter ++;
		if (stmt.m_instruction == dTreeAdressStmt::m_label) {		
			dTree<int, dListNode*>::Iterator iter1 (jumpMap);
			bool isReferenced = false;
			for (iter1.Begin(); iter1; iter1 ++) {
				dListNode* const node1 = iter1.GetKey();
				dTreeAdressStmt& stmt1 = node1->GetInfo();
				if ((stmt1.m_instruction == dTreeAdressStmt::m_goto) || (stmt1.m_instruction == dTreeAdressStmt::m_if)){
					if (stmt1.m_jmpTarget == node) {
						isReferenced = true;
						break;
					}
				}
			}
			if (!isReferenced) {
				ret = true;
				Remove(node);
				jumpMap.Remove(node);
			}
		}
	}

	// delete dead code labels
	for (iter.Begin(); iter; ) {
		dListNode* const node = iter.GetKey();
		dTreeAdressStmt& stmt = node->GetInfo();
		iter ++;
		if (stmt.m_instruction == dTreeAdressStmt::m_goto) {
			for (dListNode* deadNode = node->GetNext(); deadNode && (deadNode->GetInfo().m_instruction != dTreeAdressStmt::m_label); deadNode = node->GetNext()) {
				ret = true;
				Remove(deadNode);
			}
		}
	}

	return ret;
}

bool dCIL::RemoveNop(dListNode* const functionNode)
{
	bool ret = false;
	dCIL::dListNode* nextStmtNode;
	for (dCIL::dListNode* stmtNode = functionNode; stmtNode; stmtNode = nextStmtNode) {
		nextStmtNode = stmtNode->GetNext();
		dTreeAdressStmt& stmt = stmtNode->GetInfo();	
		if (stmt.m_instruction == dTreeAdressStmt::m_nop) {
			Remove(stmtNode);
			ret = true;
		}
	}
	return ret;
}



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


/*
void dCIL::ConvertToLLVM (llvm::Module* const module, llvm::LLVMContext &Context)
{

}
*/

void dCIL::Optimize (llvm::Function* const function)
{
    // Optimize the function.
    m_optimizer.run(*function);
}

dTreeAdressStmt::dArgType dCIL::GetType (const llvm::Type* const type) const
{
	dTreeAdressStmt::dArgType intrinsicType;

	llvm::Type::TypeID returnTypeID = type->getTypeID();
	switch (returnTypeID)
	{
		case llvm::Type::TypeID::IntegerTyID:
		{
			if (type->isIntegerTy (32)) {
				intrinsicType = dTreeAdressStmt::dArgType (dTreeAdressStmt::m_int);
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


		//VoidTyID = 0,    ///<  0: type with no size
		//HalfTyID,        ///<  1: 16-bit floating point type
		//FloatTyID,       ///<  2: 32-bit floating point type
		//DoubleTyID,      ///<  3: 64-bit floating point type
		//X86_FP80TyID,    ///<  4: 80-bit floating point type (X87)
		//FP128TyID,       ///<  5: 128-bit floating point type (112-bit mantissa)
		//PPC_FP128TyID,   ///<  6: 128-bit floating point type (two 64-bits, PowerPC)
		//LabelTyID,       ///<  7: Labels
		//MetadataTyID,    ///<  8: Metadata
	}

	return intrinsicType;
}

void dCIL::EmitFunctionDeclaration (const llvm::Function& llvmFunction)
{
	const llvm::StringRef& functionName = llvmFunction.getName();
	const llvm::Type* const returnType = llvmFunction.getReturnType();
	//const llvm::FunctionType* const functionType = llvmFuntion.getFunctionType();
	llvm::Type::TypeID returnTypeID = returnType->getTypeID();

	dTreeAdressStmt::dArgType intrinsicType = GetType (returnType);

	dCIL::dListNode* const functionNode = NewStatement();
	dTreeAdressStmt& function = functionNode->GetInfo();
	function.m_instruction = dTreeAdressStmt::m_function;
	function.m_arg0.m_label = functionName.data();
	function.m_arg0.m_type = intrinsicType;
	DTRACE_INTRUCTION (&function);

	for (llvm::Function::const_arg_iterator iter (llvmFunction.arg_begin()); iter != llvmFunction.arg_end(); iter ++) {
		const llvm::Argument* const argument = iter;
		const llvm::StringRef& name = argument->getName();

		const llvm::Type* const argType = argument->getType();
		dTreeAdressStmt::dArgType intrinsicType = GetType (argType);

		dCIL::dListNode* const argNode = NewStatement();
		dTreeAdressStmt& stmt = argNode->GetInfo();

		stmt.m_instruction = dTreeAdressStmt::m_argument;
		stmt.m_arg0.m_type = intrinsicType;
		stmt.m_arg0.m_label = name.data();
		DTRACE_INTRUCTION (&stmt);
	}
}



void dCIL::EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dTree<const dCIL::dListNode*, const llvm::BasicBlock*>& visited, dList<const dCIL::dListNode*>& terminalInstructions)
{
	if (!visited.Find (block)) {
		const dCIL::dListNode* const blockNode = EmitBasicBlockBody (function, block, terminalInstructions);
		visited.Insert (blockNode, block);

		const llvm::TerminatorInst* const teminatorIntruction = block->getTerminator();
		int successorsCount = teminatorIntruction->getNumSuccessors();
		for (int i = 0; i < successorsCount; i ++) { 
			const llvm::BasicBlock* const successorBlock = teminatorIntruction->getSuccessor(i);
			EmitBasicBlockBody (function, successorBlock, visited, terminalInstructions);
		}
	}
}

void dCIL::BuildFromLLVMFunctions (const llvm::Function& llvmFunction)
{
	// emet function decalaration
	EmitFunctionDeclaration (llvmFunction);

	// iterate over bascia block and emit teh block body
	dList<const dCIL::dListNode*> terminalInstructions;
	dTree<const dCIL::dListNode*, const llvm::BasicBlock*> visited;
	const llvm::BasicBlock* const entryBlock = &llvmFunction.getEntryBlock();
	EmitBasicBlockBody (llvmFunction, entryBlock, visited, terminalInstructions);
}


const dCIL::dListNode*dCIL::EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dList<const dCIL::dListNode*>& terminalInstructions)
{
	const llvm::StringRef& blockName = block->getName ();

	dCIL::dListNode* const blockLabelNode = NewStatement();
	dTreeAdressStmt& blockLabel = blockLabelNode->GetInfo();
	blockLabel.m_instruction = dTreeAdressStmt::m_label;
	blockLabel.m_arg0.m_label = blockName.data();
	DTRACE_INTRUCTION (&blockLabel);

	for (llvm::BasicBlock::const_iterator iter (block->begin()); iter != block->end(); iter ++) {
		const llvm::Instruction* const intruction = iter;
		int opcode = intruction->getOpcode();

		dCIL::dListNode* node = NULL;
		switch (opcode)
		{
			case llvm::Instruction::Ret:
			{
				node = EmitReturn (intruction);
				break;
			}

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

			case llvm::Instruction::Sub:
			{
				node = EmitIntegerAritmetic (intruction);
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

dCIL::dListNode* dCIL::EmitReturn (const llvm::Instruction* const intruction) const
{
	return NULL;
}

dCIL::dListNode* dCIL::EmitIntegerCompare (const llvm::Instruction* const intruction) const
{
	return NULL;
}

dCIL::dListNode* dCIL::EmitIntegerBranch (const llvm::Instruction* const intruction) const
{
//		const char *xxx = intruction->getOpcodeName();
//		const char *xxx1 = intruction->getOpcodeName();

	return NULL;
}

dCIL::dListNode* dCIL::EmitIntegerAritmetic (const llvm::Instruction* const intruction) const
{
	return NULL;
}