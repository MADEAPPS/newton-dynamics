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



class dCILInstrNop: public dCILInstr
{
	public:
	dCILInstrNop(dCIL& program);
};

class dCILInstrFunction: public dCILInstr
{
	public:
	dCILInstrFunction (dCIL& program, const dString& name, const dArgType& type);


	void Serialize (char* const textOut) const;
	dList<dArg>::dListNode* AddParameter (const dString& name, const dArgType& type); 

	dArg m_name;
	dList<dArg> m_parameters;
};



#endif