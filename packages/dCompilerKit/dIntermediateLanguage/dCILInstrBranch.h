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

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const {}

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

	dList<dCILInstr*>::dListNode* m_tagetNode;
};

class dCILInstrIF: public dCILThreeArgInstr
{
	public:
	dCILInstrIF (dCIL& program, const dString& name, const dArgType& type, const dString& target0, const dString& target1);
	void Serialize(char* const textOut) const;

	virtual dCILInstrIF* GetAsIF();
	virtual bool IsBasicBlockEnd() const;
	void SetTargets (dCILInstrLabel* const target0, dCILInstrLabel* const target1) ;

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;


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

	virtual bool IsBasicBlockEnd() const;
	virtual dCILInstrReturn* GetAsReturn();

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;
};

class dCILInstrCall: public dCILTwoArgInstr
{
	public:
	dCILInstrCall(dCIL& program, const dString& returnValue, const dArgType& type, const dString& target, dList<dArg>& parameters);
//	dCILInstrCall(dCIL& program, const dString& name, const dArgType& type, dList<dArg>& parameters);
	void Serialize(char* const textOut) const;

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;

	dList<dArg> m_parameters;
};


#endif