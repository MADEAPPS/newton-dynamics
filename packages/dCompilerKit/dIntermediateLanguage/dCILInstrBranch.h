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
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const {}
	void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph) {}
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }

	
	virtual bool IsBasicBlockBegin() const;
	virtual dCILInstrLabel* GetAsLabel();
};

class dCILInstrGoto: public dCILSingleArgInstr
{
	public:
	dCILInstrGoto(dCIL& program, const dString& label);
	void Serialize(char* const textOut) const;

	virtual bool IsBasicBlockEnd() const;
	virtual dCILInstrGoto* GetAsGoto();

	void SetTarget (dCILInstrLabel* const target0);
	dList<dCILInstr*>::dListNode* GetTarget () const;

	virtual bool ApplySemanticReordering () {return false;};
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const {}
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const {}

	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {dAssert (0);}
	dList<dCILInstr*>::dListNode* m_tagetNode;
};

class dCILInstrIFNot: public dCILThreeArgInstr
{
	public:
	dCILInstrIFNot (dCIL& program, const dString& name, const dArgType& type, const dString& target0, const dString& target1);
	void Serialize(char* const textOut) const;
	virtual void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const;

	virtual dCILInstrIFNot* GetAsIF();
	virtual bool IsBasicBlockEnd() const;
	void SetTargets (dCILInstrLabel* const target0, dCILInstrLabel* const target1) ;

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const {}

	void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}

	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { dAssert(0);  return false; }

	dList<dCILInstr*>::dListNode* GetTrueTarget () const;
	dList<dCILInstr*>::dListNode* GetFalseTarget () const;

	dList<dCILInstr*>::dListNode* GetTrue ();
	dList<dCILInstr*>::dListNode* m_tagetNode0;
	dList<dCILInstr*>::dListNode* m_tagetNode1;
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
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const {}

	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }

	void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
};

class dCILInstrCall: public dCILTwoArgInstr
{
	public:
	dCILInstrCall(dCIL& program, const dString& returnValue, const dArgType& type, const dString& target, dList<dArg>& parameters);
//	dCILInstrCall(dCIL& program, const dString& name, const dArgType& type, dList<dArg>& parameters);
	void Serialize(char* const textOut) const;
	void EmitOpcode (dVirtualMachine::dOpCode* const codeOutPtr) const;

	void SetTarget(dList<dCILInstr*>::dListNode* const node) {m_tagetNode = node; }
	//dList<dCILInstr*>::dListNode* GetFalseTarget() const;

	virtual dCILInstrCall* GetAsCall() { return this; }

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const;
	virtual void AddKilledStatements(const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const;

	virtual bool ApplyCopyPropagation(dCILInstrMove* const moveInst, dDataFlowGraph& dataFlow) const { return false; }
	virtual void AsignRegisterName(const dRegisterInterferenceGraph& interferenceGraph);

	dList<dArg> m_parameters;
	dList<dCILInstr*>::dListNode* m_tagetNode;
};


#endif