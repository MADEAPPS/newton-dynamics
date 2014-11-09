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

dCIL::dCIL()
	:dList()
	,m_mark(1)
	,m_tempIndex (0)
	,m_labelIndex (0)
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

	program->ExecuteFunction ("_Fibonacci::_Fibonacci::int", "::int" , 10);


	return program;
}


void dCIL::RegisterAllocation(dListNode* const functionNode)
{
	dAssert(0);
/*
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
*/


	//	dDataFlowGraph datFlowGraph (this, functionNode);
	//	datFlowGraph.RegistersAllocation (D_INTEGER_REGISTER_COUNT - 1);
}



void dCIL::OptimizeSSA (dListNode* const functionStart)
{
	dDataFlowGraph dataFlow (this, functionStart);

	bool pass = true;
	while (pass) {
		pass = false;
Trace();
		pass |= dataFlow.ApplyDeadCodeEliminationSSA ();
Trace();
		pass |= dataFlow.ApplyConstantPropagationSSA ();
Trace();
		pass |= dataFlow.ApplyConstantConditionalSSA();
Trace();
		pass |= dataFlow.ApplyCopyPropagationSSA ();
Trace();
		pass |= dataFlow.ApplyConditionalConstantPropagationSSA();
	}
}




