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
#include "dCILInstrMiscellaneous.h"
#include "dRegisterInterferenceGraph.h"


dString dCILInstr::m_ssaPosfix ("__ssa");

dCILInstr::dMapTable dCILInstr::m_maptable[] = 
{
	{dCILInstr::m_void, "void"}, 
	{dCILInstr::m_bool, "bool"}, 
	{dCILInstr::m_byte, "byte"}, 
	{dCILInstr::m_short, "short"}, 
	{dCILInstr::m_int, "int"}, 
	{dCILInstr::m_long, "long"}, 
	{dCILInstr::m_float, "float"}, 
	{dCILInstr::m_double, "double"}, 
	{dCILInstr::m_classPointer, "classPointer"},
	{dCILInstr::m_constInt, "constInt"},
	{dCILInstr::m_constFloat, "constFloat"}
};

void dCILInstr::dArgType::SetType (const dCILInstr::dArgType& type)
{
	m_isPointer = type.m_isPointer;
	m_intrinsicType = type.m_intrinsicType;
}



dCILInstr::dIntrisicType dCILInstr::GetTypeID (const dString& typeName)
{
	for (int i = 0; i < sizeof (m_maptable) / sizeof (m_maptable[0]); i ++) {
		if (m_maptable[i].m_name == typeName) {
			return m_maptable[i].m_intrinsicType;
		}
	}
	return dCILInstr::m_int;
}

int dCILInstr::dArgType::GetSizeInByte() const
{
	return sizeof (int);
}


dString dCILInstr::dArgType::GetTypeName () const 
{
	dAssert (m_intrinsicType < sizeof (m_maptable) / sizeof (m_maptable[0]));
	if (m_isPointer) {
		return m_maptable[m_intrinsicType].m_name + dCIL::m_pointerDecoration;
	} else {
		return m_maptable[m_intrinsicType].m_name;
	}
}


void dCILInstr::dArg::SetType (const dArgType& type)
{
	dArgType::SetType (type);
}

void dCILInstr::dArg::SetType (dIntrisicType intrinsicType, bool pointer)
{
	m_isPointer = pointer;
	m_intrinsicType = intrinsicType;
}

dCILInstr::dCILInstr (dCIL& program)
	:m_cil(&program)
	,m_myNode (NULL)
	,m_basicBlock (NULL)
{
	m_myNode = program.Append (this);
}

dCILInstr::~dCILInstr ()
{
	if (m_myNode) {
		m_cil->Remove(m_myNode);
	}
}

dString dCILInstr::MakeSSAName(const dString& name, int ssaPostfix) const
{
	return dString(name + m_ssaPosfix + dString(ssaPostfix));
}

dString dCILInstr::RemoveSSAPostfix(const dString& name) const
{
	int index = name.Find (m_ssaPosfix);
	if (index >= 0) {
		dString copy (name);
		copy[index] = 0;
		return copy;
	}
	return name;
}

void dCILInstr::Nullify()
{
	ReplaceInstruction (new dCILInstrNop(*m_cil));
}

void dCILInstr::ReplaceInstruction (dCILInstr* const newIntruction)
{
	newIntruction->GetCil()->Remove(newIntruction->GetNode());
	dCIL* const cil = m_cil;
	dList<dCILInstr*>::dListNode* const node = m_myNode;
	m_myNode = NULL;
	delete this;
	node->GetInfo() = newIntruction;
	newIntruction->m_cil = cil;
	newIntruction->m_myNode = node;
}

void dCILInstr::Serialize(char* const textOut) const
{
	textOut[0] = 0;
}

void dCILInstr::Trace() const
{
#if 1
	char text[4096];
	Serialize(text);
	dTrace ((text));
#endif
}

void dCILInstr::AddKilledStatementLow(const dArg& arg, const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const
{
	dAssert(0);

/*
	datFloatPoint.m_generateStmt = true;
	dAssert(dictionary.Find(arg.m_label));
	dList<dCIL::dListNode*>& defsList = dictionary.Find(arg.m_label)->GetInfo();
	for (dList<dCIL::dListNode*>::dListNode* defNode = defsList.GetFirst(); defNode; defNode = defNode->GetNext()) {
		dCIL::dListNode* const killStement = defNode->GetInfo();
		if (killStement != datFloatPoint.m_statement) {
			datFloatPoint.m_killStmtSet.Insert(killStement);
		}
	}
*/
}

bool dCILSingleArgInstr::DeadElimination (dDataFlowGraph& dataFlow)
{
	dAssert (0);
	return 0;
/*
	dDataFlowPoint& info = dataFlow.m_dataFlowGraph.Find(GetNode())->GetInfo();
	dDataFlowPoint::dVariableSet<dString>& liveOut = info.m_liveOutputSet;

	bool ret = false;
	const dCILInstr::dArg arg0 = m_arg0;
	//const dCILInstr::dArg arg1 = move->GetArg1();
	//if (!liveOut.Find(arg0.m_label) || (arg0.m_label == arg1.m_label)) {
	if (!liveOut.Find(arg0.m_label)) {
		ret = true;
		Nullify();
		dataFlow.UpdateLiveInputLiveOutput();
	}

	return ret;
*/
}

void dCILSingleArgInstr::AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph)
{
	dAssert (0);
//	m_arg0.m_label = interferenceGraph.GetRegisterName(m_arg0.m_label);
}

void dCILTwoArgInstr::AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph)
{
	dAssert (0);
/*
	m_arg1.m_label = interferenceGraph.GetRegisterName(m_arg1.m_label);
	dCILSingleArgInstr::AssignRegisterName(interferenceGraph);
*/
}

void dCILThreeArgInstr::AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph)
{
	dAssert (0);
/*
	if ((m_arg2.m_intrinsicType != m_constInt) && (m_arg2.m_intrinsicType != m_constFloat)) {
		m_arg2.m_label = interferenceGraph.GetRegisterName(m_arg2.m_label);
	}
	dCILTwoArgInstr::AssignRegisterName(interferenceGraph);
*/
}

