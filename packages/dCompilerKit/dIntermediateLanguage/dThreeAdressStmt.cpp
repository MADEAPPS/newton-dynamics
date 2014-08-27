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
#include "dThreeAdressStmt.h"

#ifdef _DEBUG
int dThreeAdressStmt::m_debugCount = 0;
#endif


dThreeAdressStmt::dMapTable dThreeAdressStmt::m_maptable[] = {
	{dThreeAdressStmt::m_void, "void"}, 
	{dThreeAdressStmt::m_bool, "bool"}, 
	{dThreeAdressStmt::m_byte, "byte"}, 
	{dThreeAdressStmt::m_short, "short"}, 
	{dThreeAdressStmt::m_int, "int"}, 
	{dThreeAdressStmt::m_long, "long"}, 
	{dThreeAdressStmt::m_float, "float"}, 
	{dThreeAdressStmt::m_double, "double"}, 
	{dThreeAdressStmt::m_classPointer, "classPointer"},
	{dThreeAdressStmt::m_constInt, "constInt"},
	{dThreeAdressStmt::m_constFloat, "constFloat"}
};


dThreeAdressStmt::dThreeAdressStmt(void)
	:m_instruction(m_nop)
	,m_operator(m_nothing)
	,m_arg0()
	,m_arg1()
	,m_arg2()
    ,m_trueTargetJump(NULL)
    ,m_falseTargetJump(NULL)
{
#ifdef _DEBUG
	m_debug = m_debugCount;
	m_debugCount ++;;
#endif
}

dThreeAdressStmt::~dThreeAdressStmt(void)
{
}




dThreeAdressStmt::dIntrisicType dThreeAdressStmt::GetTypeID (const dString& typeName)
{
	for (int i = 0; i < sizeof (m_maptable) / sizeof (m_maptable[0]); i ++) {
		if (m_maptable[i].m_name == typeName) {
			return m_maptable[i].m_intrinsicType;
		}
	}
	dAssert (0);
	return dThreeAdressStmt::m_int;
}

/*
dString dThreeAdressStmt::GetTypeString (const dArgType argType)
{
	dAssert (0);
	dAssert (argType.m_intrinsicType < sizeof (m_maptable) / sizeof (m_maptable[0]));
	return m_maptable[argType.m_intrinsicType].m_name;
}
*/


dString dThreeAdressStmt::dArgType::GetTypeName () const 
{
	dAssert (m_intrinsicType < sizeof (m_maptable) / sizeof (m_maptable[0]));
	if (m_isPointer) {
		return m_maptable[m_intrinsicType].m_name + dCIL::m_pointerDecoration;
	} else {
		return m_maptable[m_intrinsicType].m_name;
	}
}

void dThreeAdressStmt::dArgType::SetType (const dThreeAdressStmt::dArgType& type)
{
	m_isPointer = type.m_isPointer;
	m_intrinsicType = type.m_intrinsicType;
}


const dThreeAdressStmt::dArgType& dThreeAdressStmt::dArg::GetType () const
{
	return *this;
}

void dThreeAdressStmt::dArg::SetType (const dArgType& type)
{
	dArgType::SetType (type);
}

void dThreeAdressStmt::dArg::SetType (dIntrisicType intrinsicType, bool pointer)
{
	m_isPointer = pointer;
	m_intrinsicType = intrinsicType;
}

dString dThreeAdressStmt::GetTypeString (const dArg& arg) const
{
	dAssert (0);
//	return GetTypeString (arg.m_type);
	return "";
}

void dThreeAdressStmt::TraceAssigment (char* const text) const
{
	text[0] = 0;
	char* assignOperator = "";
	switch (m_operator)
	{
		case m_nothing:
		{
			break;
		}

		case m_equal:
		{
			//dTrace ((" = "));
			assignOperator = " = " ;
			break;
		}

		case m_add:
		{
			//dTrace ((" + "));
			assignOperator = " + " ;
			break;
		}
		case m_sub:
		{
			//dTrace ((" - "));
			assignOperator = " - " ;
			break;
		}

		case m_mul:
		{
			//dTrace ((" * "));
			assignOperator = " * " ;
			break;
		}
		case m_div:
		{
			//dTrace ((" / "));
			assignOperator = " / " ;
			break;
		}

		case m_mod:
		{
			//dTrace ((" %c ", '%'));
			assignOperator = " %% " ;
			break;
		}

		case m_identical:
		{
			//dTrace ((" == "));
			assignOperator = " == " ;
			break;
		}

		case m_different:
		{
			//dTrace ((" != "));
			assignOperator = " != " ;
			break;
		}

		case m_less:
		{
			//dTrace ((" < "));
			assignOperator = " < " ;
			break;
		}

		case m_greather:
		{
			//dTrace ((" > "));
			assignOperator = " > " ;
			break;
		}

		case m_lessEqual:
		{
			//dTrace ((" <= "));
			assignOperator = " <= " ;
			break;
		}

		case m_greatherEqual:
		{
			//dTrace ((" >= "));
			assignOperator = " >= " ;
			break;
		}
		default:;
			dAssert (0);

	}

	if (m_operator != m_nothing) {
		sprintf(text, "\t%s %s = %s %s%s%s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr(), assignOperator, m_arg2.GetTypeName().GetStr(), m_arg2.m_label.GetStr() );
	} else {
		sprintf(text, "\t%s %s = %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
	}
}

/*
void dThreeAdressStmt::TraceConditional (char* const textOut) const
{
	textOut[0] = 0;
	switch (m_operator)
	{
		case m_identical:
		{
			sprintf (textOut, "\tif (%s == %s) goto %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			break;
		}

		case m_different:
		{
			sprintf (textOut, "\tif (%s != %s) goto %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			break;
		}

		case m_less:
		{
			sprintf (textOut, "\tif (%s < %s) goto %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			break;
		}

		case m_greather:
		{
			sprintf (textOut, "\tif (%s > %s) goto %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			break;
		}

		case m_lessEqual:
		{
			sprintf (textOut, "\tif (%s <= %s) goto %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			break;
		}

		case m_greatherEqual:
		{
			sprintf (textOut, "\tif (%s >= %s) goto %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			break;
		}
	}
}
*/

void dThreeAdressStmt::Trace (char* const textOut) const
{
	textOut[0] = 0;
	switch (m_instruction)
	{
		case m_function:
		{
			//sprintf (textOut, "\nfunction %s\n", m_arg0.m_label.GetStr());
			//sprintf (textOut, "\nfunction %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
			sprintf (textOut, "\nfunction %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
			break;
		}

		case m_argument:
		{
			sprintf (textOut, "\targument %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
			break;
		}

		case m_local:
		{
			sprintf (textOut, "\tlocal %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
			break;
		}


		case m_ret:
		{
			//sprintf (textOut, "\tret %d\n", m_extraInformation);
			sprintf (textOut, "\tret %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
			break;
		}

		case m_call:
		{
			if (m_arg0.m_isPointer || (m_arg0.m_intrinsicType != m_void)) {
				sprintf (textOut, "\t%s %s = call %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
			} else {
				sprintf (textOut, "\t%s call %s\n", m_arg0.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
			}
			break;
		}

		case m_assigment:
		{
			TraceAssigment (textOut);
			break;
		}

		case m_if:
		{
			if (m_arg2.m_label != "") {
				sprintf (textOut, "\tif (%s %s) goto %s else goto %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			} else {
				sprintf (textOut, "\tif (%s %s) goto %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
			}
			break;
		}

		case m_goto:
		{
			sprintf (textOut, "\tgoto %s\n", m_arg0.m_label.GetStr());
			break;
		}

		case m_label:
		{
			sprintf (textOut, "%s:\n", m_arg0.m_label.GetStr());
			break;
		}

		case m_loadBase:
		{
			sprintf (textOut, "\t%s %s = [%s %s]\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
			break;
		}

		case m_storeBase:
		{
			sprintf (textOut, "\t[%s %s] = %s %s\n", m_arg0.GetTypeName().GetStr(),  m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
			break;
		}

		case m_param:
		{
			sprintf (textOut, "\tparam %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
			break;
		}

		case m_load:
		{
			sprintf (textOut, "\t%s %s = [%s %s + %s %s]\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr(), m_arg2.GetTypeName().GetStr(), m_arg2.m_label.GetStr());
			break;
		}

		case m_store:
		{
			sprintf (textOut, "\t[%s %s + %s %s] = %s %s\n", m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr(), m_arg2.GetTypeName().GetStr(), m_arg2.m_label.GetStr(), m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
			break;
		}

		case m_new:
		{
			sprintf (textOut, "\t%s = new %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
			break;
		}

		case m_release:
		{
			sprintf (textOut, "\trelease %s\n", m_arg0.m_label.GetStr());
			break;
		}

        case m_reference:
        {
            sprintf (textOut, "\t%s = reference %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
            break;
        }



		case m_nop:
		{
			//sprintf (textOut, "\tnop %s %d \n", m_arg2.m_label.GetStr(), m_extraInformation);
			break;
		}

		default:;
		dAssert (0);
	}
}


void dThreeAdressStmt::Trace () const
{
//	#ifdef TRACE_INTERMEDIATE_CODE
	char text[2048];
	Trace(text);
	dTrace ((text));
//	#endif
}