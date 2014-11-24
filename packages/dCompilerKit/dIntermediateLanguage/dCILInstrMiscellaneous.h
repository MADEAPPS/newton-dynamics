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

#ifndef _DCIL_INSTRUC_MISCELANEOUS_H_
#define _DCIL_INSTRUC_MISCELANEOUS_H_


#include "dCIL.h"
#include "dCILInstr.h"

class dCILInstrReturn;
class dCILInstrFunction;


class dCILInstrEnter: public dCILInstr
{
	public:
	dCILInstrEnter(dCIL& program, dCILInstrFunction* const predecessor, int registerMask, int localMemorySize);

	void Serialize(char* const textOut) const;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const;

	virtual bool ApplySemanticReordering() { return false; }
	virtual void AddGeneratedAndUsedSymbols(dDataFlowPoint& datFloatPoint) const {}
	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {dAssert (0);}
	virtual void AddDefinedVariable(dInstructionVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }
	virtual dCILInstrEnter* GetAsEnter() { return this; }

	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver) {}


	int m_registerMask;
	int m_localMemorySize;
};

class dCILInstrLeave : public dCILInstr
{
	public:
	dCILInstrLeave (dCILInstrEnter* const enter, dCILInstrReturn* const successor);

	void Serialize(char* const textOut) const;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const;

	virtual bool ApplySemanticReordering() { return false; }
	virtual void AddGeneratedAndUsedSymbols(dDataFlowPoint& datFloatPoint) const {}
	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {dAssert (0);}
	virtual void AddDefinedVariable(dInstructionVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }

	virtual dCILInstrLeave* GetAsLeave() { return this; }

	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver) {}


	dCILInstrEnter* m_enter;
	int m_registerMask;
	int m_localMemorySize;
};


class dCILInstrNop: public dCILInstr
{
	public:
	dCILInstrNop(dCIL& program);
	virtual bool ApplySemanticReordering () { return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const {}
	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {}
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};
	void Serialize(char* const textOut) const;

	virtual int GetByteCodeSize() const { return 0; }
	virtual dCILInstrNop* GetAsNop() { return this; }

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }

	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList) {}
	virtual void ReplaceArgument(const dArg& arg, const dArg& newArg) {}
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver) {}

	dString m_comment;
};

class dCILInstrFunction: public dCILInstr
{
	public:
	dCILInstrFunction (dCIL& program, const dString& name, const dArgType& type);

	void Serialize (char* const textOut) const;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const {}
	dList<dArg>::dListNode* AddParameter (const dString& name, const dArgType& type); 

	virtual dCILInstrFunction* GetAsFunction()
	{
		return this;
	}

	virtual bool ApplySemanticReordering () {return false;} 
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;
	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {dAssert (0);}
	virtual void AddDefinedVariable (dInstructionVariableDictionary& dictionary) const {dAssert (0);}
	virtual void AddKilledStatements (const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {dAssert (0);}
	virtual void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};

	virtual int GetByteCodeSize() const { return 0; }

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }

	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList) {}
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver) {}
	
	dArg m_name;
	dList<dArg> m_parameters;
};

class dCILInstrFunctionEnd : public dCILInstr
{
	public: 
	dCILInstrFunctionEnd (dCILInstrFunction* const functionBegin);

	void Serialize(char* const textOut) const;
	virtual void EmitOpcode(dVirtualMachine::dOpCode* const codeOutPtr) const {}

	virtual bool ApplySemanticReordering() { return false; }
	virtual void AddGeneratedAndUsedSymbols(dDataFlowPoint& datFloatPoint) const {}
	virtual void AddUsedVariable (dInstructionVariableDictionary& dictionary) const {dAssert (0);}
	virtual void AddDefinedVariable(dInstructionVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements(const dInstructionVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual void AssignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};

	virtual int GetByteCodeSize() const { return 0; }
	virtual dCILInstrFunctionEnd* GetAsFunctionEnd() { return this; }

	virtual bool ApplyDeadElimination (dDataFlowGraph& dataFlow) { return false; }
	virtual bool ApplyCopyPropagation (dCILInstrMove* const moveInst) { return false; }

	// ***********************
	virtual dArg* GetGeneratedVariable () { return NULL; }
	virtual void GetUsedVariables (dList<dArg*>& variablesList) {}
	virtual void ApplyConstantPropagationSSA (dConstantPropagationSolver& solver) {}

	dCILInstrFunction* m_function;
};

#endif