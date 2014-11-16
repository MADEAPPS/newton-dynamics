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

#ifndef _DCIL_INSTRUC_ARITHMETIC_H_
#define _DCIL_INSTRUC_ARITHMETIC_H_


#include "dCIL.h"
#include "dCILInstr.h"


class dCILInstrThreeArgArithmetic: public dCILThreeArgInstr
{
	public: 
	dCILInstrThreeArgArithmetic(dCIL& program, const dArg& arg0, const dArg& arg1, const dArg& arg2)
		:dCILThreeArgInstr(program, arg0, arg1, arg2)
	{
	}

	dCILInstrThreeArgArithmetic* GetAsThreeArgArithmetic()
	{
		return this;
	}

	virtual bool ApplyDeadElimination() const 
	{ 
		return true; 
	}

	// ***********************
//	virtual dArg* GetGeneratedVariable () { return &m_arg0; }
	
};

class dCILInstrIntergerLogical : public dCILInstrThreeArgArithmetic
{
	public:
	dCILInstrIntergerLogical (dCIL& program, dOperator operation, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1, const dString& name2, const dArgType& type2);

	void Serialize(char* const textOut) const;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const;

	virtual bool ApplySemanticReordering ();
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const;

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow);
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst);

	// ***********************
	virtual dArg* GetGeneratedVariable () { return &m_arg0; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList);
	virtual void ReplaceArgument (const dArg& arg, dCILInstr* const newInstruction, const dArg& newArg);

	virtual bool ApplyConstantFoldingSSA ();
	virtual bool ApplyConstantPropagationSSA (dConstantPropagationSolver& solver);

	dOperator m_operator;
};




#endif