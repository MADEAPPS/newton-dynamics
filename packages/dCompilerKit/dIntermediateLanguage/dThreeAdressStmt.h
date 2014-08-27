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

class dThreeAdressStmt
{
	public:
	enum dIntrisicType
	{
		m_void,
		m_bool,
		m_byte,
		m_short,
		m_int,
		m_long,
		m_float,
		m_double,
		m_classPointer,
		m_constInt,
		m_constFloat,
	};

	struct dMapTable 
	{
		dIntrisicType m_intrinsicType;
		dString m_name;
	};

	struct dArgType
	{
		dArgType ()
			:m_intrinsicType(m_int)
			,m_isPointer(false)
		{
		}

		dArgType (dIntrisicType intrinsicType)
			:m_intrinsicType(intrinsicType)
			,m_isPointer(false)
		{
		}

		dString GetTypeName () const;
		void SetType (const dArgType& type);

		dIntrisicType m_intrinsicType;
		bool m_isPointer;
	};

	struct dArg: public dArgType
	{
		dArg ()
			:dArgType()
			,m_label("")
		{
		}

		const dArgType& GetType () const;
		void SetType (const dArgType& type);
		void SetType (dIntrisicType intrinsicType, bool pointer);

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
		
		m_function,
		m_assigment,

		m_if, 
		m_goto,
		m_label,

		m_new,
		m_release,
        m_reference,

		m_argument,
		m_load,
		m_store,

		m_local,
		m_param,
		m_loadBase,
		m_storeBase,

		m_phi,
	};
	dThreeAdressStmt(void);
	~dThreeAdressStmt(void);

	dInstruction m_instruction;
	dOperator m_operator;
	dArg m_arg0;
	dArg m_arg1;
	dArg m_arg2;
	dList<dThreeAdressStmt>::dListNode* m_trueTargetJump;
    dList<dThreeAdressStmt>::dListNode* m_falseTargetJump;

#ifdef _DEBUG
	int m_debug;
	static int m_debugCount;
#endif

	void Trace () const;
	void Trace (char* const textOut) const;
	void TraceAssigment (char* const textOut) const;
//	void TraceConditional (char* const textOut) const;

	
	dString GetTypeString (const dArg& arg) const;
//	static dString GetTypeString (const dArgType argType);
	static dIntrisicType GetTypeID (const dString& typeName);

	static dMapTable m_maptable[];
};


#endif