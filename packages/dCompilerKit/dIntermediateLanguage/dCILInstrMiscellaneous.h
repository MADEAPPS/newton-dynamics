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
	virtual void AddDefinedVariable(dDefinedVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements(const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};
	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }
	virtual dCILInstrEnter* GetAsEnter() { return this; }

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
	virtual void AddDefinedVariable(dDefinedVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements(const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};
	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }

	virtual dCILInstrLeave* GetAsLeave() { return this; }

	dCILInstrEnter* m_enter;
	int m_registerMask;
	int m_localMemorySize;
};


class dCILInstrNop: public dCILInstr
{
	public:
	dCILInstrNop();
	dCILInstrNop(dCIL& program);
	virtual bool ApplySemanticReordering () { return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const {}
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};
	void Serialize(char* const textOut) const;

	virtual int GetByteCodeSize() const { return 0; }
	virtual dCILInstrNop* GetAsNop() { return this; }

	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }


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
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const {dAssert (0);}
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {dAssert (0);}
	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};

	virtual int GetByteCodeSize() const { return 0; }
	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }
	
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
	virtual void AddDefinedVariable(dDefinedVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements(const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {};

	virtual int GetByteCodeSize() const { return 0; }
	virtual dCILInstrFunctionEnd* GetAsFunctionEnd() { return this; }

	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }

	dCILInstrFunction* m_function;
};

#endif