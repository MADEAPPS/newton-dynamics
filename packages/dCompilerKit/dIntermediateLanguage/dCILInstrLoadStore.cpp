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


dCILInstrLocal::dCILInstrLocal (dCIL& program, const dString& name, const dArgType& type)
	:dCILSingleArgInstr (program, dArg (name, type))
{
}

void dCILInstrLocal::Serialize(char* const textOut) const
{
	sprintf (textOut, "\tlocal %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
}

void dCILInstrLocal::AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const
{
	datFloatPoint.m_generatedVariable = m_arg0.m_label;
}

void dCILInstrLocal::AddDefinedVariable (dDefinedVariableDictionary& dictionady) const  
{
	dDefinedVariableDictionary::dTreeNode* const node = dictionady.Insert (m_arg0.m_label);
	node->GetInfo().Append (m_myNode);
}

dCILInstrMove::dCILInstrMove (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1)
	:dCILTwoArgInstr (program, dArg (name0, type0), dArg (name1, type1))
{
	dAssert (0);
}

void dCILInstrMove::Serialize(char* const textOut) const
{
//	sprintf (textOut, "\tlocal %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
	sprintf(textOut, "\t%s %s = %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
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
	datFloatPoint.m_generatedVariable = m_arg0.m_label;
	datFloatPoint.m_usedVariableSet.Insert (m_arg1.m_label);
}

void dCILInstrLoad::AddDefinedVariable (dDefinedVariableDictionary& dictionady) const  
{
	dDefinedVariableDictionary::dTreeNode* const node = dictionady.Insert (m_arg0.m_label);
	node->GetInfo().Append (m_myNode);
}


dCILInstrStore::dCILInstrStore (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1)
	:dCILTwoArgInstr (program, dArg (name0, type0), dArg (name1, type1))
{
}

void dCILInstrStore::Serialize(char* const textOut) const
{
	sprintf (textOut, "\t[%s %s] = %s %s\n", m_arg0.GetTypeName().GetStr(),  m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
}

void dCILInstrStore::AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const
{
	datFloatPoint.m_usedVariableSet.Insert(m_arg0.m_label);
	datFloatPoint.m_usedVariableSet.Insert(m_arg1.m_label);
}
