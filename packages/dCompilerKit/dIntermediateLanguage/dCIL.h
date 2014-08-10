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

#ifndef __dCIL_H_
#define __dCIL_H_

#include "dCILstdafx.h"
#include "dTreeAdressStmt.h"


#define D_USE_COMPLEX_ADRESSING_MODE

#define D_TEMPRARY_SYMBOL			"t"
#define D_LOOP_HEADER_SYMBOL		"loopHeader"
#define D_LOOP_TAIL_SYMBOL			"loopTail"


inline dString GetTemporaryVariableName(int index)
{
	char tmp[256];
	sprintf (tmp, "%s%d", D_TEMPRARY_SYMBOL, index);
	return tmp;
}

inline dString GetReturnVariableName()
{
	return GetTemporaryVariableName(0);
}

class dDataFlowGraph; 

class dCIL: public dList<dTreeAdressStmt>
{
	public:
/*
	enum dReturnType
	{
		m_voidNone,
		m_intRegister,
		m_floatRegister,
	};

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
	};
*/

	class dReturnValue
	{
		public:
		dReturnValue ()
			:m_type(dTreeAdressStmt::m_int)
		{
			m_f = 0.0;
		}

		dTreeAdressStmt::dArgType m_type;
		union {;
			dMachineIntRegister m_i;
			dMachineFloatRegister m_f;
		};
	};

	
	dCIL(llvm::Module* const module);
	virtual ~dCIL(void);

	void Trace();

	dString NewTemp (); 
	dString NewLabel (); 
	void ResetTemporaries();
	dListNode* NewStatement();


    void Optimize (llvm::Function* const function);
//	void ConvertToLLVM (llvm::Module* const module, llvm::LLVMContext &Context);
//	void Optimize (dListNode* const functionNode, int argumentInRegisters, dReturnType returnType);
//	void Optimize (dListNode* const functionNode, int argumentInRegisters);
	private:
//	bool RemoveNop(dListNode* const functionNode);
//	bool RemoveRedundantJumps(dListNode* const functionNode);

	void InitializeTarget();
	void InitializeTargetMC();
	void InitializeTargetInfo();
	void InitializeAsmPrinter();
	void InitializeMCAsmParser();

	int m_mark;
	int m_tempIndex;
	int m_labelIndex;
	bool m_commutativeOperator[dTreeAdressStmt::m_operatorsCount];
	dTreeAdressStmt::dOperator m_conditionals[dTreeAdressStmt::m_operatorsCount];
	dTreeAdressStmt::dOperator m_operatorComplement[dTreeAdressStmt::m_operatorsCount];

	//dNVMTragetMacrhine m_targetMachile;
	static llvm::Target m_target;
    llvm::legacy::FunctionPassManager m_optimizer;
	
	friend dDataFlowGraph;
};


#endif