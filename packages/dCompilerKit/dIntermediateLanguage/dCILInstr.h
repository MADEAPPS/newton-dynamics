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

#ifndef  _DCIL_INSTRUC_H_
#define  _DCIL_INSTRUC_H_

/*
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

	class dArgType
	{
		public:
		dArgType ()
			:m_intrinsicType(m_int)
			,m_isPointer(false)
		{
			dAssert (0);
		}

		dArgType (dIntrisicType intrinsicType)
			:m_intrinsicType(intrinsicType)
			,m_isPointer(false)
		{
			dAssert (0);
		}

		int GetSizeInByte() const;

		dString GetTypeName () const;
		void SetType (const dArgType& type);

		dIntrisicType m_intrinsicType;
		bool m_isPointer;
	};

	class dArg: public dArgType
	{
		public:
		dArg ()
			:dArgType()
			,m_label("")
		{
			dAssert (0);
		}

		const dArgType& GetType () const
		{
			return *this;
		}

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
		m_ifnot, 
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

	void Trace () const;
	void Trace (char* const textOut) const;
	void TraceAssigment (char* const textOut) const;
//	void TraceConditional (char* const textOut) const;

	
	dString GetTypeString (const dArg& arg) const;
//	static dString GetTypeString (const dArgType argType);
	static dIntrisicType GetTypeID (const dString& typeName);

	static dMapTable m_maptable[];
};
*/

class dCIL;
class dCILInstrNop;
class dCILInstrLeave;
class dCILInstrEnter;
class dCILInstrCall;
class dCILInstrMove;
class dCILInstrGoto;
class dCILInstrLabel;
class dCILInstrReturn;
class dCILInstrArgument;
class dCILInstrFunction;
class dCILSingleArgInstr;
class dCILInstrFunctionEnd;
class dCILInstrConditional;
class dCILInstrThreeArgArithmetic;

class dDataFlowPoint;
class dDataFlowGraph;
class dRegisterInterferenceGraph;
class dDefinedVariableDictionary;

class dCILInstr
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

	class dMapTable 
	{
		public:
		dIntrisicType m_intrinsicType;
		dString m_name;
	};

	class dArgType
	{
		public:
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

		int GetSizeInByte() const;

		dString GetTypeName () const;
		void SetType (const dArgType& type);

		dIntrisicType m_intrinsicType;
		bool m_isPointer;
	};

	class dArg: public dArgType
	{
		public:
		dArg ()
			:dArgType()
			,m_label("")
		{
		}

		dArg (const dString& label, const dArgType& type)
			:dArgType(type)
			,m_label(label)
		{
		}

		const dArgType& GetType () const
		{
			return *this;
		}

		void SetType (const dArgType& type);
		void SetType (dIntrisicType intrinsicType, bool pointer);
		dString m_label;
	};


	dCILInstr (dCIL& program);
	virtual ~dCILInstr ();

	dCIL* GetCil() const { return m_cil; }
	dList<dCILInstr*>::dListNode* GetNode() const { return m_myNode; }

	virtual int GetByteCodeSize() const { return 1; }
	virtual int GetByteCodeOffset () const { return m_byteCodeOffset; }
	virtual void SetByteCodeOffset (int offset) { m_byteCodeOffset = offset; }

	

	virtual bool CanBeEliminated() const { return false; }
	virtual bool IsBasicBlockEnd() const { return false; }
	virtual bool IsBasicBlockBegin() const { return false; }

	virtual dCILInstrLeave* GetAsLeave() { return NULL; }
	virtual dCILInstrEnter* GetAsEnter() { return NULL; }
	virtual dCILInstrNop* GetAsNop() { return NULL; }
	virtual dCILInstrConditional* GetAsIF() { return NULL; }
	virtual dCILInstrMove* GetAsMove() { return NULL; }
	virtual dCILInstrCall* GetAsCall() { return NULL; }
	virtual dCILInstrGoto* GetAsGoto() { return NULL; }
	virtual dCILInstrLabel* GetAsLabel() { return NULL; }
	virtual dCILInstrReturn* GetAsReturn() { return NULL; }
	virtual dCILInstrArgument* GetAsArgument() { return NULL; }
	virtual dCILInstrFunction* GetAsFunction() { return NULL; }
	virtual dCILInstrFunctionEnd* GetAsFunctionEnd() { return NULL; }
	virtual dCILSingleArgInstr* GetAsSingleArg()  { return NULL; }
	virtual dCILInstrThreeArgArithmetic* GetAsThreeArgArithmetic() { return NULL; }


	//virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const = 0;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const 
	{
		dAssert(0); 
	}

	virtual bool ApplySemanticReordering () = 0;
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const = 0;
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const = 0;
	virtual void AddKilledStatements(const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const = 0;
	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) = 0;
	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const = 0;
	
	virtual void Serialize(char* const textOut) const;

	virtual void Trace() const;

	
	void Nullify();

	static dIntrisicType GetTypeID(const dString& typeName);

	protected:
	dCILInstr();
	virtual void AddKilledStatementLow(const dArg& arg, const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const;

	dCIL* m_cil;
	dList<dCILInstr*>::dListNode* m_myNode;
	int m_byteCodeOffset;
	static dMapTable m_maptable[];
};


class dCILSingleArgInstr: public dCILInstr
{
	public:
	dCILSingleArgInstr (dCIL& program, const dArg &arg)
		:dCILInstr (program)
		,m_arg0(arg)
	{
	}

	virtual dCILSingleArgInstr* GetAsSingleArg()  
	{ 
		return this; 
	}

	virtual bool ApplySemanticReordering ()
	{
		return false;
	}

	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);
	const dArg& GetArg0 () const 
	{
		return m_arg0;
	}

	dArg m_arg0;
};

class dCILTwoArgInstr: public dCILSingleArgInstr
{
	public:
	dCILTwoArgInstr (dCIL& program, const dArg& arg0, const dArg& arg1)
		:dCILSingleArgInstr (program, arg0)
		,m_arg1(arg1)
	{
	}

	virtual bool ApplySemanticReordering () = 0;
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const = 0;
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const = 0;
	virtual void AddKilledStatements(const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const = 0;

	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);

	const dArg& GetArg1 () const 
	{
		return m_arg1;
	}

	dArg m_arg1;
};

class dCILThreeArgInstr: public dCILTwoArgInstr
{
	public:
	enum dOperator
	{
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

	dCILThreeArgInstr (dCIL& program, const dArg& arg0, const dArg& arg1, const dArg& arg2)
		:dCILTwoArgInstr (program, arg0, arg1)
		,m_arg2(arg2)
	{
	}

	const dArg& GetArg2 () const 
	{
		return m_arg2;
	}

	virtual bool ApplySemanticReordering () = 0;
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const = 0;
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const = 0;
	virtual void AddKilledStatements(const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const = 0;
	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);

	dArg m_arg2;
};


#endif