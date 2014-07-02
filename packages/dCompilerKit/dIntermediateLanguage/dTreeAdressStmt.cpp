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
#include "dTreeAdressStmt.h"

#ifdef _DEBUG
int dTreeAdressStmt::m_debugCount = 0;
#endif


dTreeAdressStmt::dMapTable dTreeAdressStmt::m_maptable[] = {
	{dTreeAdressStmt::m_void, "void"}, 
	{dTreeAdressStmt::m_bool, "bool"}, 
	{dTreeAdressStmt::m_byte, "byte"}, 
	{dTreeAdressStmt::m_short, "short"}, 
	{dTreeAdressStmt::m_int, "int"}, 
	{dTreeAdressStmt::m_long, "long"}, 
	{dTreeAdressStmt::m_float, "float"}, 
	{dTreeAdressStmt::m_double, "double"}, 
	{dTreeAdressStmt::m_classPointer, "classPointer"}
};


dTreeAdressStmt::dTreeAdressStmt(void)
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

dTreeAdressStmt::~dTreeAdressStmt(void)
{
}

const char* dTreeAdressStmt::GetTypeString (const dArg& arg) const
{
	return m_maptable[arg.m_type].m_name.GetStr();
}

void dTreeAdressStmt::TraceAssigment (char* const text) const
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

	sprintf(text, "\t%s %s = %s%s%s\n", GetTypeString(m_arg0), m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), assignOperator, m_arg2.m_label.GetStr() );
}

/*
void dTreeAdressStmt::TraceConditional (char* const textOut) const
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

void dTreeAdressStmt::Trace (char* const textOut) const
{
	textOut[0] = 0;
	switch (m_instruction)
	{
		case m_function:
		{
			//sprintf (textOut, "\nfunction %s\n", m_arg0.m_label.GetStr());
			sprintf (textOut, "\nfunction %s %s\n", GetTypeString(m_arg0), m_arg0.m_label.GetStr());
			break;
		}

		case m_argument:
		{
			sprintf (textOut, "\targument %s %s\n", GetTypeString(m_arg0), m_arg0.m_label.GetStr());
			break;
		}

		case m_local:
		{
			sprintf (textOut, "\tlocal %s %s\n", GetTypeString(m_arg0), m_arg0.m_label.GetStr());
			break;
		}


		case m_ret:
		{
			//sprintf (textOut, "\tret %d\n", m_extraInformation);
			sprintf (textOut, "\tret %s %s\n", GetTypeString(m_arg0), m_arg0.m_label.GetStr());
			break;
		}

		case m_call:
		{
			sprintf (textOut, "\t%s %s = call %s\n", GetTypeString(m_arg0), m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
			break;
		}




		case m_assigment:
		{
			TraceAssigment (textOut);
			break;
		}

		case m_if:
		{
			sprintf (textOut, "\tif (%s) goto %s else goto %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
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
			sprintf (textOut, "\t%s %s = [%s]\n", GetTypeString(m_arg0), m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
			break;
		}

		case m_storeBase:
		{
			sprintf (textOut, "\t[%s] = %s %s\n", m_arg0.m_label.GetStr(), GetTypeString(m_arg1), m_arg1.m_label.GetStr());
			break;
		}

		case m_param:
		{
			sprintf (textOut, "\tparam %s %s\n", GetTypeString(m_arg0), m_arg0.m_label.GetStr());
			break;
		}

		case m_load:
		{
            dAssert (0);
/*
			int multiplier = 1 << m_extraInformation;
			if (multiplier != 1) {
				sprintf (textOut, "\t%s = [%s + %s * %d]\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr(), multiplier);
			} else {
				sprintf (textOut, "\t%s = [%s + %s]\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			}
*/
			break;
		}

		case m_store:
		{
            dAssert (0);
            /*
			int multiplier = 1 << m_extraInformation;
			if (multiplier != 1) {
				sprintf (textOut, "\t[%s + %s * %d] = %s\n", m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr(), multiplier, m_arg0.m_label.GetStr());
			} else {
				sprintf (textOut, "\t[%s + %s] = %s\n", m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr(), m_arg0.m_label.GetStr());
			}
*/
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


void dTreeAdressStmt::Trace () const
{
//	#ifdef TRACE_INTERMEDIATE_CODE
	char text[2048];
	Trace(text);
	dTrace ((text));
//	#endif
}