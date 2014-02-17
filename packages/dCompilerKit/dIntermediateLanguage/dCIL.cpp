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
//#include "dDataFlowGraph.h"


Target dCIL::m_target;

dCIL::dCIL(const Target &T, StringRef arch, StringRef cpu, StringRef featuresStr, TargetOptions options, Reloc::Model relocModel, CodeModel::Model CMModel, CodeGenOpt::Level optLevel)
	:LLVMTargetMachine (m_target, arch, cpu, featuresStr, options, relocModel, CMModel, optLevel)
//	:dList()
//	,m_mark(1)
//	,m_tempIndex (0)
//	,m_labelIndex (0)
{
/*
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
*/
}

dCIL::~dCIL(void)
{
}

dCIL* dCIL::CreateTargetMachine()
{
	StringRef CPU;
	StringRef Features;
	TargetOptions Options;


//	PassRegistry *Registry = PassRegistry::getPassRegistry();
//	initializeCore(*Registry);
//	initializeCodeGen(*Registry);
//	initializeLoopStrengthReducePass(*Registry);
//	initializeLowerIntrinsicsPass(*Registry);
//	initializeUnreachableBlockElimPass(*Registry);



	RegisterTarget();
	return (dCIL*)m_target.createTargetMachine(D_VIRTUAL_MACHINE_NAME, CPU, Features, Options);
}

void dCIL::RegisterTarget()
{
	if (!m_target.getName()) {
		TargetRegistry::RegisterTarget(m_target, D_VIRTUAL_MACHINE_NAME, D_VIRTUAL_MACHINE_DESC, &getArchMatch, true);

		RegisterTargetMachine<dCIL> dommy(m_target);
/*
		// Register assembler
		RegisterMCAsmInfoFn A(m_target, createLSLAsmInfo);

		// Register the MC codegen info.
		RegisterMCCodeGenInfoFn C(m_target, createLSLCodeGenInfo);

		// Register the MC instruction info.
		TargetRegistry::RegisterMCInstrInfo(m_target, createLSLInstrInfo);

		// Register the MC register info.
		TargetRegistry::RegisterMCRegInfo(m_target, createLSLRegisterInfo);

		// Register the MC subtarget info.
		TargetRegistry::RegisterMCSubtargetInfo(m_target, X86_MC::createLSLSubtargetInfo);

		// Register the MC instruction analyzer.
		TargetRegistry::RegisterMCInstrAnalysis(m_target, createLSLInstrAnalysis);

		// Register the code emitter.
		TargetRegistry::RegisterMCCodeEmitter(m_target, createLSLCodeEmitter);

		// Register the asm backend.
		TargetRegistry::RegisterMCAsmBackend(m_target, createX86_32AsmBackend);

		// Register the object streamer.
		TargetRegistry::RegisterMCObjectStreamer(m_target, createMCStreamer);

		// Register the MCInstPrinter.
		TargetRegistry::RegisterMCInstPrinter(m_target, createLSLInstPrinter);

		// Register the MC relocation info.
		TargetRegistry::RegisterMCRelocationInfo(m_target, createLSLRelocationInfo);
*/
	}
}

bool dCIL::getArchMatch(Triple::ArchType Arch) 
{
	dAssert (0);
//	return Arch == TargetArchType;
	return true;
}


/*
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



void dCIL::Optimize(dListNode* const functionNode, int argumentInRegisters, dReturnType returnType)
{
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


