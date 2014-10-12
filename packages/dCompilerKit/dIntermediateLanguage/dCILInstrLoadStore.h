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


class dCILInstrLocal: public dCILSingleArgInstr
{
	public:
	dCILInstrLocal (dCIL& program, const dString& name, const dArgType& type);
	void Serialize(char* const textOut) const;

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const;
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
};


class dCILInstrMove: public dCILTwoArgInstr
{
	public:
	dCILInstrMove (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1);

	void Serialize(char* const textOut) const;

	virtual dCILInstrMove* GetAsMove() { return this; }

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const {}
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const {dAssert (0);}
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {dAssert (0);}

	virtual bool CanBeEliminated() const
	{
		return true;
	}
};


class dCILInstrLoad: public dCILTwoArgInstr
{
	public:
	dCILInstrLoad (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1);
	void Serialize(char* const textOut) const;

	virtual bool CanBeEliminated() const 
	{
		return true;
	}

	virtual bool ApplySemanticReordering () {return false;};
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const;
	virtual void AddKilledStatements(const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const;
};

class dCILInstrStore: public dCILTwoArgInstr
{
	public:
	dCILInstrStore (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1);
	void Serialize(char* const textOut) const;

	virtual bool ApplySemanticReordering () {return false;}
	virtual void AddGeneratedAndUsedSymbols (dDataFlowPoint& datFloatPoint) const;
	virtual void AddDefinedVariable (dDefinedVariableDictionary& dictionary) const {}
	virtual void AddKilledStatements (const dDefinedVariableDictionary& dictionary, dDataFlowPoint& datFloatPoint) const {}
};




#endif