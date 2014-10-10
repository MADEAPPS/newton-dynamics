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
#include "dCILInstr.h"



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

class dCILInstr;
class dDataFlowGraph; 

//class dCIL: public dList<dThreeAdressStmt>
class dCIL: public  dList<dCILInstr*>
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

		dThreeAdressStmt::dIntrisicType m_type;
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

	void RegisterAllocation (dListNode* const functionNode);

	private:
	dString GetName (llvm::Value* const value) const;
	dCILInstr::dArgType GetType (const llvm::Type* const type) const;
	dCILInstr::dArgType GetType (const llvm::Value* const value) const;

	dCIL::dListNode* EmitLoad (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitStore (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitCall (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitReturn (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitPhiNode (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitGetElementPtr (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerAritmetic (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerCompare (const llvm::Instruction* const intruction);
	dCIL::dListNode* EmitIntegerBranch (const llvm::Instruction* const intruction);

	dCIL::dListNode* EmitFunctionDeclaration(const llvm::Function& function);
	dCIL::dListNode* EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dTree<dCIL::dListNode*, const llvm::BasicBlock*>& terminalInstructions);
	void EmitBasicBlockBody(const llvm::Function& function, const llvm::BasicBlock* const block, dTree<dCIL::dListNode*, const llvm::BasicBlock*>& visited, dTree<dCIL::dListNode*, const llvm::BasicBlock*>& terminalInstructions);

	int m_mark;
	int m_tempIndex;
	int m_labelIndex;
	bool m_commutativeOperator[dCILThreeArgInstr::m_operatorsCount];
	dCILThreeArgInstr::dOperator m_conditionals[dCILThreeArgInstr::m_operatorsCount];
	dCILThreeArgInstr::dOperator m_operatorComplement[dCILThreeArgInstr::m_operatorsCount];

    llvm::legacy::FunctionPassManager m_optimizer;
	
	public:
	static dString m_phiSource;
	static dString m_pointerSize;
	static dString m_pointerDecoration;
	static dString m_variableUndercore;
	static dString m_functionArgument;
	
	friend class dDataFlowGraph;
	friend class dCILInstrIntergerLogical;
};


#endif