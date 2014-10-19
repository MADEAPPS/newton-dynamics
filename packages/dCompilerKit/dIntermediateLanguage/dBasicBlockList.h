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

#ifndef _dBasicBlockList_H_
#define _dBasicBlockList_H_


#include "dCILstdafx.h"
#include "dCIL.h"
#include "dCILInstr.h"

class dBasicBlock
{
	public:
	dBasicBlock (dCIL::dListNode* const begin);
	void Trace() const;

	int m_mark;
	dCIL::dListNode* m_begin;
	dCIL::dListNode* m_end;
};

class dBasicBlocksList: public dList<dBasicBlock> 
{
	public:
	dBasicBlocksList();
	dBasicBlocksList(dCIL& cil, dCIL::dListNode* const functionNode);

	void Trace() const;

	void Clear ();
	void Build(dCIL& cil, dCIL::dListNode* const functionNode);
	dCIL::dListNode* m_begin;
	dCIL::dListNode* m_end;
};


#endif