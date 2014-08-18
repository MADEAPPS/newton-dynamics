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
	dCIL::dListNode* lastNode = functionNode->GetNext();
	for (; lastNode && (lastNode->GetInfo().m_instruction != dTreeAdressStmt::m_function); lastNode = lastNode->GetNext());

	// remove redundant jumps
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = functionNode; node != lastNode; node = nextNode) {
		nextNode = node->GetNext(); 
		const dTreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_goto) {
			dCIL::dListNode* const prevNode = node->GetPrev();
			const dTreeAdressStmt& prevStmt = prevNode->GetInfo();
			if (prevStmt.m_instruction == dTreeAdressStmt::m_ret) {
				cil.Remove(node);
			}
		}
	}

	// find the root of all basic blocks leaders
	for (dCIL::dListNode* node = functionNode; node != lastNode; node = node->GetNext()) {
		const dTreeAdressStmt& stmt = node->GetInfo();

		if (stmt.m_instruction == dTreeAdressStmt::m_label) {
			Append(dBasicBlock(node));
		}
	}

	for (dList<dBasicBlock>::dListNode* blockNode = GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();

		for (dCIL::dListNode* stmtNode = block.m_begin; !block.m_end && stmtNode; stmtNode = stmtNode->GetNext()) {
			const dTreeAdressStmt& stmt = stmtNode->GetInfo();
			switch (stmt.m_instruction)
			{
				case dTreeAdressStmt::m_if:
				case dTreeAdressStmt::m_goto:
				case dTreeAdressStmt::m_ret:
					block.m_end = stmtNode;
					break;
			}
		} 
	}
}

