/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
#include "dCILInstrLoadStore.h"
#include "dConstantPropagationSolver.h"

dCILInstrArgument::dCILInstrArgument(dCIL& program, const dString& name, const dArgType& type)
	:dCILSingleArgInstr(program, dArg(name, type))
{
}

void dCILInstrArgument::Serialize(char* const textOut) const
{
	sprintf(textOut, "\targument %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
}


void dCILInstrArgument::AddGeneratedAndUsedSymbols(dDataFlowPoint& datFloatPoint) const
{
	dAssert(0);
//	datFloatPoint.m_generatedVariable = m_arg0.m_label;
}

void dCILInstrArgument::AddDefinedVariable(dInstructionVariableDictionary& dictionary) const 
{ 
	dAssert(0);
//	dInstructionVariableDictionary::dTreeNode* const node = dictionary.Insert(m_arg0.m_label);
//	node->GetInfo().Append(m_myNode);
}

bool dCILInstrArgument::ApplyDeadElimination (dDataFlowGraph& dataFlow) 
{
	return DeadElimination (dataFlow);
}


dCILInstrLocal::dCILInstrLocal (dCIL& program, const dString& name, const dArgType& type)
	:dCILSingleArgInstr (program, dArg (name, type))
{
	dAssert(0);
}


void dCILInstrLocal::Serialize(char* const textOut) const
{
	dAssert(0);
	sprintf (textOut, "\tlocal %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
}

void dCILInstrLocal::AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const
{
	dAssert(0);
//	datFloatPoint.m_generatedVariable = m_arg0.m_label;
}

void dCILInstrLocal::AddDefinedVariable (dInstructionVariableDictionary& dictionary) const  
{
	dAssert(0);
/*
	dInstructionVariableDictionary::dTreeNode* const node = dictionary.Insert (m_arg0.m_label);
	node->GetInfo().Append (m_myNode);
*/
}

dCILInstrMove::dCILInstrMove (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1)
	:dCILTwoArgInstr (program, dArg (name0, type0), dArg (name1, type1))
{
}

void dCILInstrMove::Serialize(char* const textOut) const
{
	sprintf(textOut, "\t%s %s = %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
}

void dCILInstrMove::AddGeneratedAndUsedSymbols(dDataFlowPoint& datFloatPoint) const
{
	dAssert(0);
/*
	datFloatPoint.m_generatedVariable = m_arg0.m_label;
	dAssert(m_arg1.GetType().m_intrinsicType != m_constInt);
	datFloatPoint.m_usedVariableSet.Insert(m_arg1.m_label);
*/
}

void dCILInstrMove::AddDefinedVariable(dInstructionVariableDictionary& dictionary) const
{ 
	dAssert(0);
/*
	dInstructionVariableDictionary::dTreeNode* const node = dictionary.Insert(m_arg0.m_label);
	node->GetInfo().Append(m_myNode);
*/
}


bool dCILInstrMove::ApplyCopyPropagation (dCILInstrMove* const moveInst) 
{ 
	bool ret = false;
	if (moveInst->m_arg0.m_label == m_arg1.m_label) {
		ret = true;
		m_arg1.m_label = moveInst->m_arg1.m_label;
	}
	
	dAssert(ret);
	return ret;
}

void dCILInstrMove::AddUsedVariable (dInstructionVariableDictionary& dictionary) const
{
	dAssert(0);
/*
	dInstructionVariableDictionary::dTreeNode* const node = dictionary.Insert(m_arg1.m_label);
	node->GetInfo().Append(m_myNode);
*/
}


void dCILInstrMove::AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const
{
	dAssert(0);
//	dCILInstr::AddKilledStatementLow(m_arg0, dictionary, datFloatPoint);
}


bool dCILInstrMove::ApplyDeadElimination (dDataFlowGraph& dataFlow)
{ 
	dAssert(0);
	return 0;
/*
	if (m_arg0.m_label == m_arg1.m_label) {
		Nullify();
		dataFlow.UpdateLiveInputLiveOutput();
		return true;
	}
	return DeadElimination (dataFlow);
*/
}

void dCILInstrMove::GetUsedVariables (dList<dArg*>& variablesList)
{
	if (m_arg1.m_isPointer) {
		variablesList.Append(&m_arg1);
	} else {
		switch (m_arg1.GetType().m_intrinsicType) 
		{
			case m_constInt:
			case m_constFloat:
				break;

			default:
				variablesList.Append(&m_arg1);
		}
	}
}

void dCILInstrMove::ReplaceArgument (const dArg& arg, const dArg& newArg)
{
	if (arg.m_label == m_arg1.m_label) {
		m_arg1 = newArg;
	}
}

/*
bool dCILInstrMove::ApplyConstantPropagationSSA (dWorkList& workList, dStatementBlockDictionary& usedVariablesDictionary)
{
//Trace();
	bool ret = false;
	if ((m_arg1.GetType().m_intrinsicType == dCILInstr::m_constInt) || (m_arg1.GetType().m_intrinsicType == dCILInstr::m_constFloat)) {
		dStatementBlockDictionary::dTreeNode* const node = usedVariablesDictionary.Find(m_arg0.m_label);
		if (node) {
			dStatementBlockBucket::Iterator iter (node->GetInfo());
			for (iter.Begin(); iter; iter ++) {
				dCILInstr* const instrution = iter.GetKey()->GetInfo();
				//instrution->Trace();
				instrution->ReplaceArgument (m_arg0, NULL, m_arg1);
				//instrution->Trace();
				workList.Insert(instrution->GetNode());
			}
		}
		Nullify();
		ret = true;
	}
	return ret;
}
*/

void dCILInstrMove::ApplyConstantPropagationSSA (dConstantPropagationSolver& solver)
{
	//dAssert(solver.m_variablesList.Find(m_arg0.m_label));
	//dConstantPropagationSolver::dVariable& variable = solver.m_variablesList.Find(m_arg0.m_label)->GetInfo();

	dConstantPropagationSolver::dVariable::dValueTypes type = dConstantPropagationSolver::dVariable::m_undefined;
	dString value ("");
	if ((m_arg1.GetType().m_intrinsicType == m_constInt) || (m_arg1.GetType().m_intrinsicType == m_constFloat)) {
		type = dConstantPropagationSolver::dVariable::m_constant;
		value = m_arg1.m_label;
		//variable.m_value = dConstantPropagationSolver::dVariable::m_constant;
		//variable.m_constValue = m_arg1.m_label;
	} else {
		dConstantPropagationSolver::dVariable& variable = solver.m_variablesList.Find(m_arg1.m_label)->GetInfo();
		type = variable.m_type;
		value = variable.m_constValue;
	}

	if (type != dConstantPropagationSolver::dVariable::m_undefined) {
		solver.UpdateLatice (m_arg0, value, type);
	}
}

bool dCILInstrMove::ApplyCopyPropagationSSA (dWorkList& workList, dStatementBlockDictionary& usedVariablesDictionary)
{
	bool ret = false;
	if (!((m_arg1.GetType().m_intrinsicType == dCILInstr::m_constInt) || (m_arg1.GetType().m_intrinsicType == dCILInstr::m_constFloat))) {
		dStatementBlockDictionary::dTreeNode* const node = usedVariablesDictionary.Find(m_arg0.m_label);
		if (node) {
			dStatementBlockBucket::Iterator iter(node->GetInfo());
			for (iter.Begin(); iter; iter++) {
				dCILInstr* const instrution = iter.GetKey()->GetInfo();
				instrution->ReplaceArgument(m_arg0, m_arg1);
				workList.Insert(instrution);
			}
		}
		ret = true;
	}
	return ret;
}


void dCILInstrMove::EmitOpcode(dVirtualMachine::dOpCode* const codeOutPtr) const
{
	dVirtualMachine::dOpCode& code = codeOutPtr[m_byteCodeOffset];
	code.m_type3.m_opcode = unsigned (dVirtualMachine::m_mov);
	code.m_type3.m_reg0 = RegisterToIndex(m_arg0.m_label);
	code.m_type3.m_reg1 = RegisterToIndex(m_arg1.m_label);
	code.m_type3.m_imm3 = 0;
}


dCILInstrLoad::dCILInstrLoad (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1)
	:dCILTwoArgInstr (program, dArg (name0, type0), dArg (name1, type1))
{
}

void dCILInstrLoad::Serialize(char* const textOut) const
{
	sprintf (textOut, "\t%s %s = [%s %s]\n", m_arg0.GetTypeName().GetStr(),  m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
}

void dCILInstrLoad::AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const
{
	dAssert(0);
/*
	datFloatPoint.m_generatedVariable = m_arg0.m_label;
	datFloatPoint.m_usedVariableSet.Insert (m_arg1.m_label);
*/
}

void dCILInstrLoad::GetUsedVariables (dList<dArg*>& variablesList)
{
	variablesList.Append(&m_arg1);
}

void dCILInstrLoad::AddDefinedVariable (dInstructionVariableDictionary& dictionary) const  
{
	dAssert(0);
/*
	dInstructionVariableDictionary::dTreeNode* const node = dictionary.Insert (m_arg0.m_label);
	node->GetInfo().Append (m_myNode);
*/
}

void dCILInstrLoad::AddUsedVariable (dInstructionVariableDictionary& dictionary) const 
{
	dAssert(0);
/*
	dInstructionVariableDictionary::dTreeNode* const node = dictionary.Insert(m_arg1.m_label);
	node->GetInfo().Append(m_myNode);
*/
}

void dCILInstrLoad::AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const 
{ 
	dCILInstr::AddKilledStatementLow(m_arg0, dictionary, datFloatPoint);
}

bool dCILInstrLoad::ApplyDeadElimination (dDataFlowGraph& dataFlow)
{
	return DeadElimination (dataFlow);
}

dCILInstrStore::dCILInstrStore (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1)
	:dCILTwoArgInstr (program, dArg (name0, type0), dArg (name1, type1))
{
}

void dCILInstrStore::Serialize(char* const textOut) const
{
	sprintf (textOut, "\t[%s %s] = %s %s\n", m_arg0.GetTypeName().GetStr(),  m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
}

bool dCILInstrStore::ApplyCopyPropagation (dCILInstrMove* const moveInst) 
{ 
	dAssert(0);  
	return false; 
}

void dCILInstrStore::AddUsedVariable (dInstructionVariableDictionary& dictionary) const 
{
	dAssert(0);
/*
	dInstructionVariableDictionary::dTreeNode* const node0 = dictionary.Insert(m_arg0.m_label);
	node0->GetInfo().Append(m_myNode);

	dInstructionVariableDictionary::dTreeNode* const node1 = dictionary.Insert(m_arg1.m_label);
	node1->GetInfo().Append(m_myNode);
*/
}


void dCILInstrStore::GetUsedVariables(dList<dArg*>& variablesList)
{
	variablesList.Append(&m_arg0);
	variablesList.Append(&m_arg1);
}


void dCILInstrStore::AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const
{
	dAssert(0);
//	datFloatPoint.m_usedVariableSet.Insert(m_arg0.m_label);
//	datFloatPoint.m_usedVariableSet.Insert(m_arg1.m_label);
}


dCILInstrPhy::dCILInstrPhy (dCIL& program, const dString& name, const dArgType& type, const dBasicBlock* const basicBlock, dList<const dBasicBlock*>& sources)
	:dCILSingleArgInstr(program, dArg(name, type))
{
	m_basicBlock = (dBasicBlock*)basicBlock;
	for (dList<const dBasicBlock*>::dListNode* node = sources.GetFirst(); node; node = node->GetNext()) {
		m_sources.Append (dArgPair (m_arg0, node->GetInfo()));
	}
}


void dCILInstrPhy::Serialize(char* const textOut) const
{
	sprintf(textOut, "\t%s %s = phi (", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
	for (dList<dArgPair>::dListNode* node = m_sources.GetFirst(); node; node = node->GetNext()) {
		char tmp[1024]; 

		dArgPair& pair = node->GetInfo();
		dCILInstrLabel* const block = pair.m_block->m_begin->GetInfo()->GetAsLabel();
		dAssert(block);
		if (node->GetNext()) {
			sprintf(tmp, "[%s %s, %s], ", pair.m_arg.GetTypeName().GetStr(), pair.m_arg.m_label.GetStr(), block->GetArg0().m_label.GetStr());
		} else {
			sprintf(tmp, "[%s %s, %s]", pair.m_arg.GetTypeName().GetStr(), pair.m_arg.m_label.GetStr(), block->GetArg0().m_label.GetStr());
		}
		strcat(textOut, tmp);
	}
	strcat(textOut, ")\n");
}

void dCILInstrPhy::GetUsedVariables (dList<dArg*>& variablesList)
{
	for (dList<dArgPair>::dListNode* node = m_sources.GetFirst(); node; node = node->GetNext()) {
		dArg* const arg = &node->GetInfo().m_arg;
		if (arg->GetType().m_isPointer) {
			variablesList.Append(arg);
		} else {
			switch (arg->GetType().m_intrinsicType) 
			{
				case m_constInt:
				case m_constFloat:
					break;

				default:
					variablesList.Append(arg);
			}
		}
	}
}


void dCILInstrPhy::ReplaceArgument(const dArg& arg, const dArg& newArg)
{
	for (dList<dArgPair>::dListNode* node = m_sources.GetFirst(); node; node = node->GetNext()) {
		dArgPair& pair = node->GetInfo();
		if (arg.m_label == pair.m_arg.m_label) {
			pair.m_arg.m_label = newArg.m_label;
			break;
		}
	}
}

/*
bool dCILInstrPhy::ApplyConstantPropagationSSA (dWorkList& workList, dStatementBlockDictionary& usedVariablesDictionary)
{
	bool ret = false;
	if (!m_sources.GetFirst()->GetInfo().m_intructionNode) {
		const dArg& arg = m_sources.GetFirst()->GetInfo().m_arg;
		if ((arg.GetType().m_intrinsicType == dCILInstr::m_constInt) || (arg.GetType().m_intrinsicType == dCILInstr::m_constFloat)) {
			ret = true;
			for (dList<dArgPair>::dListNode* node = m_sources.GetFirst(); node; node = node->GetNext()) {
				const dArg& srcArg = node->GetInfo().m_intructionNode ? *node->GetInfo().m_intructionNode->GetInfo()->GetGeneratedVariable() : node->GetInfo().m_arg;
				ret &= ((srcArg.m_label == arg.m_label) && !node->GetInfo().m_intructionNode);
			}
		}
	}

	if (ret) {
		Trace();
		dAssert (0);
	}

	return ret;
}
*/

void dCILInstrPhy::ApplyConstantPropagationSSA (dConstantPropagationSolver& solver)
{
//dAssert (0);
/*
	dList<dArg> constList;
	for (dList<dArgPair>::dListNode* node = m_sources.GetFirst(); node; node = node->GetNext()) {
		const dArg* const arg = node->GetInfo().m_intructionNode ? node->GetInfo().m_intructionNode->GetInfo()->GetGeneratedVariable() : &node->GetInfo().m_arg;

		if ((arg->GetType().m_intrinsicType == dCILInstr::m_constInt) || (arg->GetType().m_intrinsicType == dCILInstr::m_constFloat)) {
			constList.Append(*arg);
		} else {
			dAssert(solver.m_variablesList.Find(arg->m_label));
			dConstantPropagationSolver::dVariable& variable = solver.m_variablesList.Find(arg->m_label)->GetInfo();
			if (variable.m_type == dConstantPropagationSolver::dVariable::m_variableValue) {
				solver.UpdateLatice (m_arg0, m_arg0.m_label, dConstantPropagationSolver::dVariable::m_variableValue);
				return; 
			}
			if (variable.m_type != dConstantPropagationSolver::dVariable::m_undefined) {
				dAssert (variable.m_type == dConstantPropagationSolver::dVariable::m_constant);
				constList.Append (dArg (variable.m_constValue, arg->GetType()));
			}
		}
	}

	if (constList.GetCount()) {
		bool equal = true;
		const dString& value = constList.GetFirst()->GetInfo().m_label; 
		for (dList<dArg>::dListNode* node = constList.GetFirst()->GetNext(); node; node = node->GetNext()) {
			equal &= (value == node->GetInfo().m_label);
		}
		if (equal) {
			solver.UpdateLatice (m_arg0, value, dConstantPropagationSolver::dVariable::m_constant);
		} else {
			solver.UpdateLatice (m_arg0, value, dConstantPropagationSolver::dVariable::m_variableValue);
		}
	}
*/	

}