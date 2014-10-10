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
#include "dCILInstrArithmetic.h"

/*
dCILInstrLocal::dCILInstrLocal (dCIL& program, const dString& name, const dArgType& type)
	:dCILSingleArgInstr (program, dArg (name, type))
{
}

void dCILInstrLocal::Serialize(char* const textOut) const
{
	sprintf (textOut, "\tlocal %s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr());
}

dCILInstrStore::dCILInstrStore (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1)
	:dCILTwoArgInstr (program, dArg (name0, type0), dArg (name1, type1))
{
}

void dCILInstrStore::Serialize(char* const textOut) const
{
	sprintf (textOut, "\t[%s %s] = %s %s\n", m_arg0.GetTypeName().GetStr(),  m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr());
}


dCILInstrLoad::dCILInstrLoad (dCIL& program, const dString& name, const dArgType& type)
	:dCILInstr (program)
	,m_source (name, type)
{
	m_destination.m_label = program.NewTemp();
	m_destination.SetType (m_source);
}

const dCILInstr::dArg& dCILInstrLoad::GetResult() const
{
	return m_destination;
}

void dCILInstrLoad::Serialize(char* const textOut) const
{
	sprintf (textOut, "\t%s %s = [%s %s]\n", m_destination.GetTypeName().GetStr(), m_destination.m_label.GetStr(), m_source.GetTypeName().GetStr(), m_source.m_label.GetStr());
}
*/

dCILInstrIntergerLogical::dCILInstrIntergerLogical (dCIL& program, dOperator operation, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1, const dString& name2, const dArgType& type2)
	:dCILThreeArgInstr (program, dArg (name0, type0), dArg (name1, type1), dArg (name2, type2))
	,m_operator(operation)
{

}

void dCILInstrIntergerLogical::Serialize(char* const textOut) const
{
	char* assignOperator = "";
	switch (m_operator)
	{
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

	sprintf(textOut, "\t%s %s = %s %s%s%s %s\n", m_arg0.GetTypeName().GetStr(), m_arg0.m_label.GetStr(), m_arg1.GetTypeName().GetStr(), m_arg1.m_label.GetStr(), assignOperator, m_arg2.GetTypeName().GetStr(), m_arg2.m_label.GetStr() );
}



bool dCILInstrIntergerLogical::ApplyRemanticReordering ()
{
	const dArg& arg1 = GetArg1();
	switch (arg1.GetType().m_intrinsicType)
	{
		case m_constInt:
		{
			dAssert (0);
			if (m_cil->m_commutativeOperator[m_operator]) {
//				dAssert (stmt.m_arg2.GetType().m_intrinsicType != dThreeAdressStmt::m_constInt);
//				dSwap (stmt.m_arg1, stmt.m_arg2);
			} else {
/*
				dCIL::dListNode* const node = m_cil->NewStatement();
				m_cil->InsertAfter (stmtNode, node);

				dThreeAdressStmt& tmpStmt = node->GetInfo();
				tmpStmt = stmt;
								
				stmt.m_instruction = dThreeAdressStmt::m_assigment;
				stmt.m_operator = dThreeAdressStmt::m_nothing;
				stmt.m_arg0.m_label += dCIL::m_variableUndercore; 
				stmt.m_arg2.m_label = "";
				tmpStmt.m_arg1 = stmt.m_arg0;
*/
			}
			return true;
		}
		default:
			break;
	}
	return false;
}
