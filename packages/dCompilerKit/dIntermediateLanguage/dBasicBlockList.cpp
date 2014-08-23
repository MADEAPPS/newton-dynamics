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
#include "dBasicBlockList.h"

dBasicBlock::dBasicBlock (dCIL::dListNode* const begin)
	:m_mark (0)
	,m_begin (begin)
	,m_end(NULL)
{
}

//void Trace() const;

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

/*
void Trace() const
{
	#ifdef TRACE_INTERMEDIATE_CODE
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			node->GetInfo().Trace();
		}
	#endif
}
*/


void dBasicBlocksList::Clear ()
{
	RemoveAll();
}

void dBasicBlocksList::Build(dCIL& cil, dCIL::dListNode* const functionNode)
{
	m_begin = functionNode;
	m_end = functionNode->GetNext();
	for (; m_end && (m_end->GetInfo().m_instruction != dThreeAdressStmt::m_function); m_end = m_end->GetNext());

	// remove redundant jumps
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = m_begin; node != m_end; node = nextNode) {
		nextNode = node->GetNext(); 
		const dThreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dThreeAdressStmt::m_goto) {
			dCIL::dListNode* const prevNode = node->GetPrev();
			const dThreeAdressStmt& prevStmt = prevNode->GetInfo();
			if (prevStmt.m_instruction == dThreeAdressStmt::m_ret) {
				cil.Remove(node);
			}
		}
	}

	// find the root of all basic blocks leaders
	for (dCIL::dListNode* node = m_begin; node != m_end; node = node->GetNext()) {
		const dThreeAdressStmt& stmt = node->GetInfo();

		if (stmt.m_instruction == dThreeAdressStmt::m_label) {
			Append(dBasicBlock(node));
		}
	}

	for (dList<dBasicBlock>::dListNode* blockNode = GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();

		for (dCIL::dListNode* stmtNode = block.m_begin; !block.m_end && stmtNode; stmtNode = stmtNode->GetNext()) {
			const dThreeAdressStmt& stmt = stmtNode->GetInfo();
			switch (stmt.m_instruction)
			{
				case dThreeAdressStmt::m_if:
				case dThreeAdressStmt::m_goto:
				case dThreeAdressStmt::m_ret:
					block.m_end = stmtNode;
					break;
			}
		} 
	}
}

