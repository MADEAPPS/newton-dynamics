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

dTreeAdressStmt::dTreeAdressStmt(void)
	:m_instruction(m_nop)
	,m_operator(m_nothing)
	,m_arg0()
	,m_arg1()
	,m_arg2()
	,m_extraInformation(0)
	,m_jmpTarget (NULL)
{
#ifdef _DEBUG
	m_debug = m_debugCount;
	m_debugCount ++;;
#endif
}

dTreeAdressStmt::~dTreeAdressStmt(void)
{
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
			_ASSERTE (0);

	}

	sprintf(text, "\t%s = %s%s%s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), assignOperator, m_arg2.m_label.GetStr() );
}

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

void dTreeAdressStmt::Trace (char* const textOut) const
{
	textOut[0] = 0;
	switch (m_instruction)
	{
		case m_enter:
		{
			sprintf (textOut, "\tenter %d\n", m_extraInformation);
			break;
		}

		case m_leave:
		{
			sprintf (textOut, "\tleave %d\n", m_extraInformation);
			break;
		}


		case m_function:
		{
			sprintf (textOut, "\nfunction %s\n", m_arg0.m_label.GetStr());
			break;
		}

		case m_assigment:
		{
			TraceAssigment (textOut);
			break;
		}

		case m_if:
		{
			TraceConditional (textOut);
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
			sprintf (textOut, "\t%s = [%s]\n", m_arg0.m_label.GetStr(), m_arg2.m_label.GetStr());
			break;
		}

		case m_storeBase:
		{
			sprintf (textOut, "\t[%s] = %s\n", m_arg0.m_label.GetStr(), m_arg2.m_label.GetStr());
			break;
		}


		case m_argument:
		{
			sprintf (textOut, "\targument %s\t; passed on the stack frame %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
			break;
		}

		case m_push:
		{
			sprintf (textOut, "\tpush %s\n", m_arg0.m_label.GetStr());
			break;
		}

		case m_pop:
		{
			sprintf (textOut, "\tpop %s\n", m_arg0.m_label.GetStr());
			break;
		}

		case m_load:
		{
			int multiplier = 1 << m_extraInformation;
			if (multiplier != 1) {
				sprintf (textOut, "\t%s = [%s + %s * %d]\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr(), multiplier);
			} else {
				sprintf (textOut, "\t%s = [%s + %s]\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr());
			}
			break;
		}

		case m_store:
		{
			int multiplier = 1 << m_extraInformation;
			if (multiplier != 1) {
				sprintf (textOut, "\t[%s + %s * %d] = %s\n", m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr(), multiplier, m_arg0.m_label.GetStr());
			} else {
				sprintf (textOut, "\t[%s + %s] = %s\n", m_arg1.m_label.GetStr(), m_arg2.m_label.GetStr(), m_arg0.m_label.GetStr());
			}
			break;
		}

		case m_call:
		{
			sprintf (textOut, "\tcall %s\n", m_arg0.m_label.GetStr());
			break;
		}
		
		case m_alloc:
		{
			sprintf (textOut, "\t%s = alloc %s\n", m_arg0.m_label.GetStr(), m_arg1.m_label.GetStr());
			break;
		}

		case m_free:
		{
			sprintf (textOut, "\tfree %s\n", m_arg0.m_label.GetStr());
			break;
		}


		case m_ret:
		{
			sprintf (textOut, "\tret %d\n", m_extraInformation);
			break;
		}

		case m_nop:
		{
			//sprintf (textOut, "\tnop %s %d \n", m_arg2.m_label.GetStr(), m_extraInformation);
			break;
		}

		default:;
		_ASSERTE (0);
	}
}


void dTreeAdressStmt::Trace () const
{
	#ifdef TRACE_INTERMEDIATE_CODE
		char text[2048];
		Trace(text);
		dTrace ((text));
	#endif
}