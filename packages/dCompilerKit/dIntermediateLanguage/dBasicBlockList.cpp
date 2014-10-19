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


#include "dCILstdafx.h"
#include "dCILInstrBranch.h"
#include "dBasicBlockList.h"

dBasicBlock::dBasicBlock (dCIL::dListNode* const begin)
	:m_mark (0)
	,m_begin (begin)
	,m_end(NULL)
{
}

void dBasicBlock::Trace() const
{
	for (dCIL::dListNode* node = m_begin; node != m_end; node = node->GetNext()) {
		node->GetInfo()->Trace();
	}
	dTrace (("\n"));
}

dBasicBlocksList::dBasicBlocksList()
	:dList<dBasicBlock> ()
{
}

dBasicBlocksList::dBasicBlocksList(dCIL& cil, dCIL::dListNode* const functionNode)
	:dList<dBasicBlock> ()
	,m_begin(NULL)
	,m_end(NULL)
{
	Build (cil, functionNode);
}


void dBasicBlocksList::Clear ()
{
	RemoveAll();
}

void dBasicBlocksList::Build(dCIL& cil, dCIL::dListNode* const functionNode)
{
	m_begin = functionNode->GetNext();
	m_end = functionNode->GetNext();
	for ( ;!m_end->GetInfo()->GetAsFunctionEnd(); m_end = m_end->GetNext());

	// remove redundant jumps
	for (dCIL::dListNode* node = m_begin; node != m_end; node = node->GetNext()) {
		dCILInstrReturn* const returnInst = node->GetInfo()->GetAsReturn();
		if (returnInst && node->GetNext()) {
			dCIL::dListNode* const nextNode = node->GetNext();
			dCILInstrGoto* const gotoInst = nextNode->GetInfo()->GetAsGoto();
			if (gotoInst) {
				dAssert (0);
				delete gotoInst;
			}
		}
	}

	// find the root of all basic blocks leaders
	for (dCIL::dListNode* node = m_begin; node != m_end; node = node->GetNext()) {
		if (node->GetInfo()->IsBasicBlockBegin()) {
			Append(dBasicBlock(node));
		}
	}

	for (dList<dBasicBlock>::dListNode* blockNode = GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		for (dCIL::dListNode* node = block.m_begin; !block.m_end && node; node = node->GetNext()) {
			if (node->GetInfo()->IsBasicBlockEnd()) {
				block.m_end = node->GetNext();
			}
		} 
	}

//	cil.Trace();
//	Trace();
}

void dBasicBlocksList::Trace() const
{
	for (dList<dBasicBlock>::dListNode* blockNode = GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		block.Trace();
	}
}