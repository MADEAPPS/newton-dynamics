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
#include "dThreeAdressStmt.h"



#define D_TEMPRARY_SYMBOL			"t"

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

class dCIL: public dList<dThreeAdressStmt>
{
	public:
	class dReturnValue
	{
		public:
		dReturnValue ()
			:m_type(dThreeAdressStmt::m_int)
		{
			m_f = 0.0;
		}

		dThreeAdressStmt::dArgType m_type;
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

	void ConvertLLVMFunctionToNVMFunction (const llvm::Function& funtion);
    void Optimize (llvm::Function* const function);

	private:
	dString GetName (llvm::Value* const value) const;
	dThreeAdressStmt::dArgType GetType (const llvm::Type* const type) const;
	dThreeAdressStmt::dArgType GetType (const llvm::Value* const value) const;

	dCIL::dListNode* EmitCall (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitReturn (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerAritmetic (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerCompare (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerBranch (const llvm::Instruction* const intruction);

	dCIL::dListNode* EmitFunctionDeclaration(const llvm::Function& function);
	const dCIL::dListNode* EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dList<dCIL::dListNode*>& terminalInstructions);
	void EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dTree<const dCIL::dListNode*, const llvm::BasicBlock*>& visited, dList<dCIL::dListNode*>& terminalInstructions);

	void RegisterAllocation (dListNode* const functionNode);
	
	

	int m_mark;
	int m_tempIndex;
	int m_labelIndex;
	bool m_commutativeOperator[dThreeAdressStmt::m_operatorsCount];
	dThreeAdressStmt::dOperator m_conditionals[dThreeAdressStmt::m_operatorsCount];
	dThreeAdressStmt::dOperator m_operatorComplement[dThreeAdressStmt::m_operatorsCount];

    llvm::legacy::FunctionPassManager m_optimizer;
	
	public:
	static dString m_pointerSize;
	static dString m_pointerDecoration;
	static dString m_variableUndercore;
	friend dDataFlowGraph;
};


#endif