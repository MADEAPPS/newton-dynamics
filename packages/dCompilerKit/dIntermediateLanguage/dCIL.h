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
#include "dTreeAdressStmt.h"

#define D_USE_COMPLEX_ADRESSING_MODE



#define D_TEMPRARY_SYMBOL			"t"
#define D_LOOP_HEADER_SYMBOL		"loopHeader"
#define D_LOOP_TAIL_SYMBOL			"loopTail"


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

class dCIL: public dList<dTreeAdressStmt>
{
	public:
	enum dReturnType
	{
		m_void,
		m_intRegister,
		m_floatRegister,
		m_classPointer,
	};
	
	dCIL(void);
	virtual ~dCIL(void);

	void Trace();

	dString NewTemp (); 
	dString NewLabel (); 
	void ResetTemporaries();
	dListNode* NewStatement();
	
	void Optimize (dListNode* const functionNode, int argumentInRegisters, dReturnType returnType);
	private:

	bool RemoveNop(dListNode* const functionNode);
	bool RemoveRedundantJumps(dListNode* const functionNode);

	int m_mark;
	int m_tempIndex;
	int m_labelIndex;
	bool m_commutativeOperator[dTreeAdressStmt::m_operatorsCount];
	dTreeAdressStmt::dOperator m_conditionals[dTreeAdressStmt::m_operatorsCount];
	dTreeAdressStmt::dOperator m_operatorComplement[dTreeAdressStmt::m_operatorsCount];

	friend dDataFlowGraph;
};


#endif