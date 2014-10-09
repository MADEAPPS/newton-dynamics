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
#include "dCILInstrBranch.h"


dCILInstrLabel::dCILInstrLabel(dCIL& program, const dString& label)
	:dCILSingleArgInstr (program, dArg (label, dArgType()))
{
}

void dCILInstrLabel::Serialize(char* const textOut) const
{
	sprintf (textOut, "%s:\n", m_arg0.m_label.GetStr());
}

dCILInstrGoto::dCILInstrGoto(dCIL& program, const dString& label)
	:dCILSingleArgInstr (program, dArg (label, dArgType()))
	,m_tagetNode(NULL)
{
}

void dCILInstrGoto::Serialize(char* const textOut) const
{
	sprintf (textOut, "\tgoto %s\n", m_arg0.m_label.GetStr());
}

void dCILInstrGoto::SetTarget (dCILInstrLabel* const target)
{
	m_tagetNode = target->GetNode();
	dAssert (target->GetArg0().m_label == GetArg0().m_label);
}


dCILInstrIF::dCILInstrIF(dCIL& program, const dString& name, const dArgType& type, const dString& target0, const dString& target1)
	:dCILThreeArgInstr (program, dArg (name, type), dArg (target0, dArgType()), dArg (target1, dArgType()))
	,m_tagetNode0(NULL)
	,m_tagetNode1(NULL)
{
}

void dCILInstrIF::Serialize(char* const textOut) const
{
	sprintf (textOut, "\tif (%s %s) goto %s else goto %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
}


void dCILInstrIF::SetTargets (dCILInstrLabel* const target0, dCILInstrLabel* const target1)
{
	dAssert (target0->GetArg0().m_label == GetArg1().m_label);
	dAssert (target1->GetArg0().m_label == GetArg2().m_label);

	m_tagetNode0 = target0->GetNode();
	m_tagetNode1 = target1->GetNode();
}


dCILInstrReturn::dCILInstrReturn(dCIL& program, const dString& name, const dArgType& type)
	:dCILSingleArgInstr (program, dArg (name, type))
{
}

void dCILInstrReturn::Serialize(char* const textOut) const
{
	sprintf (textOut, "\tret %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
}

/*
dCILInstrCall::dCILInstrCall(dCIL& program, const dString& name, const dArgType& type, dList<dArg>& parameters)
	:dCILSingleArgInstr (program, dArg (name, type))
{
	for (dList<dArg>::dListNode* node = parameters.GetFirst(); node; node = node->GetNext()) {
		m_parameters.Append (node->GetInfo());
	}
}
*/

dCILInstrCall::dCILInstrCall(dCIL& program, const dString& returnValue, const dArgType& type, const dString& target, dList<dArg>& parameters)
	:dCILTwoArgInstr (program, dArg (returnValue, type), dArg (target, dArgType()))
{
	for (dList<dArg>::dListNode* node = parameters.GetFirst(); node; node = node->GetNext()) {
		m_parameters.Append (node->GetInfo());
	}
}

void dCILInstrCall::Serialize(char* const textOut) const
{
	if (m_arg0.m_isPointer || (m_arg0.m_intrinsicType != m_void)) {
		sprintf (textOut, "\t%s %s = call %s (", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
	} else {
		sprintf (textOut, "\t%s call %s (", m_arg0.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
	}

	for (dList<dArg>::dListNode* node = m_parameters.GetFirst(); node; node = node->GetNext()) { 
		char text[2048];
		const dArg& arg = node->GetInfo();
		if (node->GetNext()) {
			sprintf (text, "%s %s, ", arg.GetTypeName().GetStr(), arg.m_label.GetStr());
		} else {
			sprintf (text, "%s %s", arg.GetTypeName().GetStr(), arg.m_label.GetStr());
		}
		strcat (textOut, text);
	}
	strcat (textOut, ")\n");
}
