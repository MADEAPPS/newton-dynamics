/* Copyright (c) <2003-2016> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _DCIL_INSTRUC_BRANCH_H_
#define _DCIL_INSTRUC_BRANCH_H_


#include "dCIL.h"
#include "dCILInstr.h"


class dCILInstrLabel: public dCILSingleArgInstr
{
	public:
	dCILInstrLabel(dCIL& program, const dString& label);
	void Serialize(char* const textOut) const;
	virtual int GetByteCodeSize() const { return 0; }
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const {}

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const {}
	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {}
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const {}
	void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {}
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }
	virtual bool IsBasicBlockBegin() const;
	virtual dCILInstrLabel* GetAsLabel();

	virtual bool IsDefineOrUsedVariable() { return false; }

	// ***********************
	virtual dArg* GetGeneratedVariable () {return NULL;}
	virtual void GetUsedVariables (dList<dArg*>& variablesList) {}
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver) {}
};

class dCILInstrGoto: public dCILSingleArgInstr
{
	public:
	dCILInstrGoto(dCIL& program, const dString& label);
	void Serialize(char* const textOut) const;

	virtual bool IsBasicBlockEnd() const;
	virtual dCILInstrGoto* GetAsGoto();

	void SetLabel (const dString& label);
	void SetTarget (dCILInstrLabel* const target0);
	dList<dCILInstr*>::dListNode* GetTarget () const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {}
	virtual bool ApplySemanticReordering () {return false;};
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const {}
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const {}

	void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {}
	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	dList<dCILInstr*>::dListNode* m_tagetNode;

	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList) {}
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver);

	virtual bool IsDefineOrUsedVariable() { return false; }

};

class dCILInstrConditional: public dCILThreeArgInstr
{
	public:
	enum dBranchMode
	{
		m_if,
		m_ifnot,
	};

	dCILInstrConditional (dCIL& program, dBranchMode mode, const dString& name, const dArgType& type, const dString& target0, const dString& target1);
	void Serialize(char* const textOut) const;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const;

	virtual dCILInstrConditional* GetAsIF();
	virtual bool IsBasicBlockEnd() const;
	void SetLabels (const dString& label0, const dString& label1);
	void SetTargets (dCILInstrLabel* const target0, dCILInstrLabel* const target1);

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const {}

	void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { dAssert(0);  return false; }

	dList<dCILInstr*>::dListNode* GetTrueTarget () const;
	dList<dCILInstr*>::dListNode* GetFalseTarget () const;


	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList);
	virtual void ReplaceArgument (const dArg& arg, const dArg& newArg);
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver);

	dBranchMode m_mode;
	dList<dCILInstr*>::dListNode* m_targetNode0;
	dList<dCILInstr*>::dListNode* m_targetNode1;
};


class dCILInstrReturn: public dCILSingleArgInstr
{
	public:
	dCILInstrReturn(dCIL& program, const dString& name, const dArgType& type);
	void Serialize(char* const textOut) const;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const; 

	virtual int GetByteCodeSize() const;
	virtual bool IsBasicBlockEnd() const;
	virtual dCILInstrReturn* GetAsReturn();

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const {}

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }

	void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}

	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList);

};

class dCILInstrCall: public dCILTwoArgInstr
{
	public:
//	dCILInstrCall(dCIL& program, const dString& returnValue, const dArgType& type, const dString& target, dList<dArg>& parameters);
	dCILInstrCall(dCIL& program, const dString& returnValue, const dArgType& type, const dString& functionName);
	void Serialize(char* const textOut) const;
	void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const;

	void AddArgument (const dArg& argument);
	void SetTarget(dList<dCILInstr*>::dListNode* const node) {m_tagetNode = node; }

	virtual dCILInstrCall* GetAsCall() { return this; }

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;

	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const;
	virtual void AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const;

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }
	virtual void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);

	// ***********************
	virtual dArg* GetGeneratedVariable ();
	virtual void GetUsedVariables (dList<dArg*>& variablesList);

	dList<dArg> m_parameters;
	dList<dCILInstr*>::dListNode* m_tagetNode;
};

#endif