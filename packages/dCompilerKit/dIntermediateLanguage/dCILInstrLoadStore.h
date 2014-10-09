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
};

class dCILInstrStore: public dCILTwoArgInstr
{
	public:
	dCILInstrStore (dCIL& program, const dString& name0, const dArgType& type0, const dString& name1, const dArgType& type1);

	void Serialize(char* const textOut) const;
};


class dCILInstrLoad: public dCILInstr
{
	public:
	dCILInstrLoad (dCIL& program, const dString& name, const dArgType& type);

	void Serialize(char* const textOut) const;

	const dArg& GetResult() const; 

	dArg m_source;
	dArg m_destination;
};



#endif