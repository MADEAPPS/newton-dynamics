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

#ifndef  __dTreeAdressStmt_H_
#define  __dTreeAdressStmt_H_

#ifdef _DEBUG
	#define TRACE_INTERMEDIATE_CODE
#endif

#ifdef TRACE_INTERMEDIATE_CODE
	#define DTRACE_INTRUCTION(x) (x)->Trace();
#else
	#define DTRACE_INTRUCTION(x)
#endif

class dTreeAdressStmt
{
	public:
	enum dArgType
	{
		m_intVar,
		m_floatVar,
		m_intConst,
		m_floatConst,
		m_classPointer,
	};

	struct dArg
	{
		dArg ()
			:m_type (m_intVar)
			,m_label("")
		{
		}
		dArgType m_type;
		dString m_label;
	};


	enum dOperator
	{
		m_nothing,
		m_add,
		m_sub,
		m_mul,
		m_div,
		m_mod,
		m_equal,
		m_identical,
		m_different,
		m_less,
		m_lessEqual,
		m_greather,
		m_greatherEqual,
		m_operatorsCount,
	};

	enum dInstruction
	{
		m_nop,
		m_call,
        m_ret,
		m_new,
		m_release,
        m_reference,
		m_if, 
		m_enter,
		m_leave,
		m_function,
		m_argument,
		m_load,
		m_store,
		m_assigment,
		m_goto,
		m_label,
		m_loadBase,
		m_storeBase,
		m_push,
		m_pop,
	};
	dTreeAdressStmt(void);
	~dTreeAdressStmt(void);

	dInstruction m_instruction;
	dOperator m_operator;
	dArg m_arg0;
	dArg m_arg1;
	dArg m_arg2;
	int m_extraInformation;
	dList<dTreeAdressStmt>::dListNode* m_jmpTarget;

#ifdef _DEBUG
	int m_debug;
	static int m_debugCount;
#endif

	void Trace () const;
	void Trace (char* const textOut) const;
	void TraceAssigment (char* const textOut) const;
	void TraceConditional (char* const textOut) const;
};


#endif