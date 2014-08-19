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
//#define D_LOOP_HEADER_SYMBOL		"loopHeader"
//#define D_LOOP_TAIL_SYMBOL		"loopTail"


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

	void Clear();
	void Trace();

	dString NewTemp (); 
	dString NewLabel (); 
	void ResetTemporaries();
	dListNode* NewStatement();

	void ConvertLLVMFunctionToNVMFuntion (const llvm::Function& funtion);
    void Optimize (llvm::Function* const function);

	private:
	dString GetName (llvm::Value* const value) const;
	dTreeAdressStmt::dArgType GetType (const llvm::Type* const type) const;
	dTreeAdressStmt::dArgType GetType (const llvm::Value* const value) const;

	dCIL::dListNode* EmitCall (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitReturn (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerAritmetic (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerCompare (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerBranch (const llvm::Instruction* const intruction);

	dCIL::dListNode* EmitFunctionDeclaration(const llvm::Function& function);
	const dCIL::dListNode* EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dList<dCIL::dListNode*>& terminalInstructions);
	void EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dTree<const dCIL::dListNode*, const llvm::BasicBlock*>& visited, dList<dCIL::dListNode*>& terminalInstructions);

	void RegisterAllocation (dListNode* const functionNode, int argumentInRegisters);
//	bool RemoveNop(dListNode* const functionNode);
//	bool RemoveRedundantJumps(dListNode* const functionNode);

	int m_mark;
	int m_tempIndex;
	int m_labelIndex;
	bool m_commutativeOperator[dTreeAdressStmt::m_operatorsCount];
	dTreeAdressStmt::dOperator m_conditionals[dTreeAdressStmt::m_operatorsCount];
	dTreeAdressStmt::dOperator m_operatorComplement[dTreeAdressStmt::m_operatorsCount];

    llvm::legacy::FunctionPassManager m_optimizer;
	
	static dString m_variableUndercore;
	friend dDataFlowGraph;
};


#endif