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
#include "dCILInstrMiscellaneous.h"
#include "dRegisterInterferenceGraph.h"

//dString dCILInstr::m_ssaPosfix ("_ssa");
dString dCILInstr::m_ssaPosfix ("_");

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
	{dCILInstr::m_constFloat, "constFloat"},
	{dCILInstr::m_luaType, "luaType"}
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
	,m_basicBlock(NULL)
	,m_myNode (NULL)
	,m_byteCodeOffset(0)
	,m_uniqueId(program.GetInstructionUniqurID())
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
	int uniqueId = m_uniqueId;
	dList<dCILInstr*>::dListNode* const node = m_myNode;
	m_myNode = NULL;
	delete this;
	node->GetInfo() = newIntruction;
	newIntruction->m_cil = cil;
	newIntruction->m_uniqueId = uniqueId;
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

const char* dCILInstr::GetOperatorString(dOperator operatotion) const
{
	char* assignOperator = "";
	switch (operatotion)
	{
		case m_equal:
		{
			assignOperator = "=";
			break;
		}

		case m_add:
		{
			assignOperator = "+";
			break;
		}
		case m_sub:
		{
			assignOperator = "-";
			break;
		}

		case m_mul:
		{
			assignOperator = "*";
			break;
		}
		case m_div:
		{
			assignOperator = "/";
			break;
		}

		case m_mod:
		{
			assignOperator = "%%";
			break;
		}

		case m_identical:
		{
			assignOperator = "==";
			break;
		}

		case m_different:
		{
			assignOperator = "!=";
			break;
		}

		case m_less:
		{
			assignOperator = "<";
			break;
		}

		case m_greather:
		{
			assignOperator = ">";
			break;
		}

		case m_lessEqual:
		{
			assignOperator = "<=";
			break;
		}

		case m_greatherEqual:
		{
			assignOperator = ">=";
			break;
		}
		default:;
			dAssert(0);
	}

	return assignOperator;
}



void dCILSingleArgInstr::AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph)
{
	m_arg0.m_label = interferenceGraph.GetRegisterName(m_arg0.m_label);
}

void dCILTwoArgInstr::AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph)
{
	if ((m_arg1.m_intrinsicType != m_constInt) && (m_arg1.m_intrinsicType != m_constFloat)) {
		m_arg1.m_label = interferenceGraph.GetRegisterName(m_arg1.m_label);
	}
	dCILSingleArgInstr::AssignRegisterName(interferenceGraph);
}

void dCILThreeArgInstr::AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph)
{
	if ((m_arg2.m_intrinsicType != m_constInt) && (m_arg2.m_intrinsicType != m_constFloat)) {
		m_arg2.m_label = interferenceGraph.GetRegisterName(m_arg2.m_label);
	}
	dCILTwoArgInstr::AssignRegisterName(interferenceGraph);
}


