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
#include "dCILInstrLoadStore.h"


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

bool dCILInstrMove::ApplyConstantPropagationSSA (dWorkList& workList, dVariablesDictionary& usedVariablesDictionary)
{
//Trace();
	bool ret = false;
	if ((m_arg1.GetType().m_intrinsicType == dCILInstr::m_constInt) || (m_arg1.GetType().m_intrinsicType == dCILInstr::m_constFloat)) {
		dVariablesDictionary::dTreeNode* const node = usedVariablesDictionary.Find(m_arg0.m_label);
		if (node) {
			dStatementBucket::Iterator iter (node->GetInfo());
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

bool dCILInstrMove::ApplyCopyPropagationSSA (dWorkList& workList, dVariablesDictionary& usedVariablesDictionary)
{
	bool ret = false;
	if (!((m_arg1.GetType().m_intrinsicType == dCILInstr::m_constInt) || (m_arg1.GetType().m_intrinsicType == dCILInstr::m_constFloat))) {
		dVariablesDictionary::dTreeNode* const node = usedVariablesDictionary.Find(m_arg0.m_label);
		if (node) {
//Trace();
			dStatementBucket::Iterator iter(node->GetInfo());
			for (iter.Begin(); iter; iter++) {
				dCILInstr* const instrution = iter.GetKey()->GetInfo();
//instrution->Trace();
				instrution->ReplaceArgument(m_arg0, NULL, m_arg1);
//instrution->Trace();
				workList.Insert(instrution->GetNode());
			}
		}
		Nullify();
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


dCILInstrPhy::dCILInstrPhy (dCIL& program, const dString& name, const dArgType& type, dList<dCILInstr*>& source)
	:dCILSingleArgInstr(program, dArg(name, type))
{
	for (dList<dCILInstr*>::dListNode* node = source.GetFirst(); node; node = node->GetNext()) {
		m_sources.Append(node->GetInfo()->GetNode());
	}
}


void dCILInstrPhy::Serialize(char* const textOut) const
{
	sprintf(textOut, "\t%s %s = phy (", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
	for (dList<dArgPair>::dListNode* node = m_sources.GetFirst(); node; node = node->GetNext()) {
		dArg* const arg = node->GetInfo().m_intructionNode ? node->GetInfo().m_intructionNode->GetInfo()->GetGeneratedVariable() : &node->GetInfo().m_arg;
		dAssert (arg);
		char tmp[1024]; 
		if (node->GetNext()) {
			sprintf(tmp, "%s %s, ", arg->GetTypeName().GetStr(), arg->m_label.GetStr());
		}
		else {
			sprintf(tmp, "%s %s", arg->GetTypeName().GetStr(), arg->m_label.GetStr());
		}
		strcat(textOut, tmp);
	}
	strcat(textOut, ")\n");
}

void dCILInstrPhy::GetUsedVariables (dList<dArg*>& variablesList)
{
	for (dList<dArgPair>::dListNode* node = m_sources.GetFirst(); node; node = node->GetNext()) {
		dArg* const arg = node->GetInfo().m_intructionNode ? node->GetInfo().m_intructionNode->GetInfo()->GetGeneratedVariable() : &node->GetInfo().m_arg;
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

void dCILInstrPhy::ReplaceArgument (const dArg& arg, dCILInstr* const newInstruction, const dArg& newArg)
{
	for (dList<dArgPair>::dListNode* node = m_sources.GetFirst(); node; node = node->GetNext()) {
		dArg* const srcArg = node->GetInfo().m_intructionNode ? node->GetInfo().m_intructionNode->GetInfo()->GetGeneratedVariable() : &node->GetInfo().m_arg;
		if (arg.m_label == srcArg->m_label) {
			node->GetInfo().m_arg = newArg;
			node->GetInfo().m_intructionNode = newInstruction ? newInstruction->GetNode() : NULL;
		}
	}
}

bool dCILInstrPhy::ApplyConstantPropagationSSA (dWorkList& workList, dVariablesDictionary& usedVariablesDictionary)
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