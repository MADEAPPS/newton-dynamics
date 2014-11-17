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

#ifndef _DCIL_INSTRUC_LOAD_STORE_H_
#define _DCIL_INSTRUC_LOAD_STORE_H_


#include "dCIL.h"
#include "dCILInstr.h"



class dCILInstrArgument: public dCILSingleArgInstr
{
	public:
	dCILInstrArgument(dCIL& program, const dString& name, const dArgType& type);

	void Serialize(char* const textOut) const;
	virtual dCILInstrArgument* GetAsArgument() { return this; }

	virtual bool ApplySemanticReordering() { return false; }
	virtual void AddGeneratedAndUsedSymbols(dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {}
	virtual void AddDefinedVariable(dInstructionVariableDictionary& dictionary) const;
	virtual void AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {};

	virtual int GetByteCodeSize() const { return 0; }

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) ;
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }

	// ***********************
	virtual dArg* GetGeneratedVariable () { return &m_arg0; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList) {}
};


class dCILInstrLocal: public dCILSingleArgInstr
{
	public:
	dCILInstrLocal (dCIL& program, const dString& name, const dArgType& type);
	void Serialize(char* const textOut) const;

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {dAssert (0);}
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }
};


class dCILInstrMove: public dCILTwoArgInstr
{
	public:
	dCILInstrMove (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1);
	
	void Serialize(char* const textOut) const;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const;

	virtual dCILInstrMove* GetAsMove() { return this; }
	virtual bool ApplyDeadElimination() const { return true; }
	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols(dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const; 
	virtual void AddDefinedVariable(dInstructionVariableDictionary& dictionary) const;
	virtual void AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const;

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow)  ;
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst);

	// ***********************
	virtual dArg* GetGeneratedVariable () { return &m_arg0; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList);
	virtual bool ApplyCopyPropagationSSA (dWorkList& workList, dStatementBlockDictionary& usedVariablesDictionary);
	virtual bool ApplyConstantPropagationSSA (dWorkList& workList, dStatementBlockDictionary& usedVariablesDictionary);
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver);
};


class dCILInstrPhy: public dCILSingleArgInstr
{
	public:
	class dArgPair
	{
		public:
		dArgPair (dList<dCILInstr*>::dListNode* const intructionNode)
			:m_intructionNode(intructionNode)
			,m_block(m_intructionNode->GetInfo()->GetBasicBlock()) 
			,m_arg (*intructionNode->GetInfo()->GetGeneratedVariable())
		{
			dAssert (m_block);
		}

		dArgPair (const dArgPair& copy)
			:m_intructionNode(copy.m_intructionNode)
			,m_block (copy.m_block)
			,m_arg (copy.m_arg)
		{
			dAssert (m_block);
			dAssert (m_block == m_intructionNode->GetInfo()->GetBasicBlock());
		}
		
		dList<dCILInstr*>::dListNode* m_intructionNode;
		dBasicBlock* m_block;
		dArg m_arg;
	};

	dCILInstrPhy (dCIL& program, const dString& name0, const dArgType& type0, dList<dCILInstr*>& sources, const dBasicBlock* const basicBlock);

	void Serialize(char* const textOut) const;
	virtual void EmitOpcode(dVirtualMachine::dOpCode* const codeOutPtr) const {}

	virtual dCILInstrPhy* GetAsPhy() { return this; }
	virtual bool ApplyDeadElimination() const { return true; }
	virtual bool ApplySemanticReordering() { return false; }
	virtual void AddGeneratedAndUsedSymbols(dDataFlowPoint& datFloatPoint) const {dAssert(0);}

	virtual void AddUsedVariable(dInstructionVariableDictionary& dictionary) const {dAssert(0);}
	virtual void AddDefinedVariable(dInstructionVariableDictionary& dictionary) const {dAssert(0);}
	virtual void AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {dAssert(0);}

	virtual bool ApplyDeadElimination(dDataFlowGraph& dataFlow) {dAssert(0); return false;}
	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst) {dAssert(0); return false;}

	// ***********************
	virtual dArg* GetGeneratedVariable () { return &m_arg0; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList);
	virtual void ReplaceArgument (const dArg& arg, dCILInstr* const newInstruction, const dArg& newArg);
	virtual bool ApplyConstantPropagationSSA (dWorkList& workList, dStatementBlockDictionary& usedVariablesDictionary);
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver);

	dList<dArgPair> m_sources;
};


class dCILInstrLoad: public dCILTwoArgInstr
{
	public:
	dCILInstrLoad (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1);
	void Serialize(char* const textOut) const;

	virtual bool ApplyDeadElimination() const 
	{
		return true;
	}

	virtual bool ApplySemanticReordering () {return false;};
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const;

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow);
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { dAssert(0);  return false; }

	// ***********************
	virtual dArg* GetGeneratedVariable () { return &m_arg0; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList);
};

class dCILInstrStore: public dCILTwoArgInstr
{
	public:
	dCILInstrStore (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1);
	void Serialize(char* const textOut) const;

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) {return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst);

	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList);
};

#endif