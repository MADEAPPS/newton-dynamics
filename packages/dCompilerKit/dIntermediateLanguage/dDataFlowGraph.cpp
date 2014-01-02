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
#include "dCIL.h"
#include "dDataFlowGraph.h"


#define D_OPTIMIZE_REDUNDNACE_COPIES_ON_ARRAYS

dDataFlowGraph::dDataFlowGraph (dCIL* const cil, dCIL::dListNode* const function, dCIL::dReturnType returnType)
	:m_mark (0)
	,m_registersUsedMask(0)
	,m_cil (cil)
	,m_returnType(returnType)
	,m_returnVariableName(GetReturnVariableName())
	,m_function(function)
{
}

dDataFlowGraph::~dDataFlowGraph(void)
{
}


void dDataFlowGraph::BuildBasicBlockGraph()
{
	// build leading block map table
	m_basicBlocks.RemoveAll();
	m_dataFlowGraph.RemoveAll();

	dTree<dList<dBasicBlock>::dListNode*, dCIL::dListNode*> blocksMap;

	dList<dBasicBlock>::dListNode* const root = m_basicBlocks.Append(dBasicBlock(m_function));
	blocksMap.Insert(root, m_function);
	dBasicBlock* last = &root->GetInfo();

	for (dCIL::dListNode* node = m_function->GetNext(); node; node = node->GetNext()) {
		const dTreeAdressStmt& stmt = node->GetInfo();
		switch (stmt.m_instruction) 
		{
			case dTreeAdressStmt::m_label:
			{
				last->m_end = node->GetPrev();
				dList<dBasicBlock>::dListNode* const root = m_basicBlocks.Append(dBasicBlock(node));
				blocksMap.Insert(root, node);
				last = &root->GetInfo();
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				dCIL::dListNode* const nextNode = node->GetNext();
				const dTreeAdressStmt& stmt = nextNode->GetInfo();
				if (stmt.m_instruction != dTreeAdressStmt::m_label) {
					last->m_end = node;
					dList<dBasicBlock>::dListNode* const root = m_basicBlocks.Append(dBasicBlock(nextNode));
					blocksMap.Insert(root, nextNode);
					last = &root->GetInfo();
				}
				break;
			}

			default:
			{
				const dTreeAdressStmt& prevInst = node->GetPrev()->GetInfo();
				if ((prevInst.m_instruction == dTreeAdressStmt::m_if) && !blocksMap.Find(node)) {
					last->m_end = node->GetPrev();
					dList<dBasicBlock>::dListNode* const root = m_basicBlocks.Append(dBasicBlock(node));
					blocksMap.Insert(root, node);
					last = &root->GetInfo();
				}
				break;
			}
		}
	}

	last->m_end = m_cil->GetLast();
	while (last->m_end->GetInfo().m_instruction != dTreeAdressStmt::m_ret) {
		last->m_end = last->m_end->GetPrev();
	}
	dAssert (last->m_end->GetInfo().m_instruction == dTreeAdressStmt::m_ret);


	dTree<dBasicBlock*, dCIL::dListNode*> blockMap;
	for (dList<dBasicBlock>::dListNode* blockNode = m_basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();

		blockMap.Insert(&block, block.m_begin);

		int test = 1;
		for (dCIL::dListNode* stmtNode = block.m_begin; test; stmtNode = stmtNode->GetNext()) {
			test = (stmtNode != block.m_end);
			dTree<dDataFlowPoint, dCIL::dListNode*>::dTreeNode* const graphNode = m_dataFlowGraph.Insert(stmtNode);
			graphNode->GetInfo().Init(stmtNode);
		} 

		for (dCIL::dListNode* stmtNode = block.m_begin; stmtNode != block.m_end; stmtNode = stmtNode->GetNext()) {
			dDataFlowPoint& graphStatement = m_dataFlowGraph.Find(stmtNode)->GetInfo();
			graphStatement.m_successors.Append(&m_dataFlowGraph.Find(stmtNode->GetNext())->GetInfo());
		}

		for (dCIL::dListNode* stmtNode = block.m_end; stmtNode != block.m_begin; stmtNode = stmtNode->GetPrev()) {
			dDataFlowPoint& graphStatement = m_dataFlowGraph.Find(stmtNode)->GetInfo();
			graphStatement.m_predecessors.Append(&m_dataFlowGraph.Find(stmtNode->GetPrev())->GetInfo());
		}
	}

	m_mark += 1;  
	dList<dBasicBlock*> stack;
	stack.Append(&m_basicBlocks.GetFirst()->GetInfo());
	while (stack.GetCount()) {
		dBasicBlock* const block = stack.GetLast()->GetInfo();

		stack.Remove(stack.GetLast()->GetInfo());
		if (block->m_mark < m_mark) {
			block->m_mark = m_mark;
			m_traversalBlocksOrder.Addtop(block);

			const dTreeAdressStmt& stmt = block->m_end->GetInfo();

			if (stmt.m_instruction == dTreeAdressStmt::m_if) {
				dAssert (m_dataFlowGraph.Find(block->m_end));
				dAssert (m_dataFlowGraph.Find(stmt.m_jmpTarget));
				dAssert (blockMap.Find(stmt.m_jmpTarget));
				dAssert (m_dataFlowGraph.Find(block->m_end->GetNext()));
				dAssert (blockMap.Find(block->m_end->GetNext()));

				dDataFlowPoint* const graphStatement = &m_dataFlowGraph.Find(block->m_end)->GetInfo();
				dDataFlowPoint* const child0 = &m_dataFlowGraph.Find(stmt.m_jmpTarget)->GetInfo();
				dDataFlowPoint* const child1 = &m_dataFlowGraph.Find(block->m_end->GetNext())->GetInfo();

				graphStatement->m_successors.Append(child0);
				graphStatement->m_successors.Append(child1);
				child0->m_predecessors.Append(graphStatement);
				child1->m_predecessors.Append(graphStatement);

				stack.Append(blockMap.Find(stmt.m_jmpTarget)->GetInfo());
				stack.Append(blockMap.Find(block->m_end->GetNext())->GetInfo());

			} else if (stmt.m_instruction == dTreeAdressStmt::m_goto) {
				dAssert (m_dataFlowGraph.Find(block->m_end));
				dAssert (m_dataFlowGraph.Find(stmt.m_jmpTarget));
				dAssert (blockMap.Find(stmt.m_jmpTarget));

				dDataFlowPoint* const graphStatement = &m_dataFlowGraph.Find(block->m_end)->GetInfo();
				dDataFlowPoint* const child0 = &m_dataFlowGraph.Find(stmt.m_jmpTarget)->GetInfo();

				graphStatement->m_successors.Append(child0);
				child0->m_predecessors.Append(graphStatement);

				stack.Append(blockMap.Find(stmt.m_jmpTarget)->GetInfo());

			} else if (stmt.m_instruction != dTreeAdressStmt::m_ret) {
				dAssert (m_dataFlowGraph.Find(block->m_end));
				dAssert (m_dataFlowGraph.Find(block->m_end->GetNext()));
				dAssert (blockMap.Find(block->m_end->GetNext()));

				dDataFlowPoint* const graphStatement = &m_dataFlowGraph.Find(block->m_end)->GetInfo();
				dDataFlowPoint* const child0 = &m_dataFlowGraph.Find(block->m_end->GetNext())->GetInfo();

				graphStatement->m_successors.Append(child0);
				child0->m_predecessors.Append(graphStatement);

				stack.Append(blockMap.Find(block->m_end->GetNext())->GetInfo());
			}
		}
	}
}


void dDataFlowGraph::dBasicBlock::Trace() const
{
	#ifdef TRACE_INTERMEDIATE_CODE
		int test = 1;
		for (dCIL::dListNode* stmtNode = m_begin; test; stmtNode = stmtNode->GetNext()) {
			test = (stmtNode != m_end);
			const dTreeAdressStmt& stmt = stmtNode->GetInfo();
			DTRACE_INTRUCTION(&stmt);
		}
		dTrace(("\n"));
	#endif
}


void dDataFlowGraph::BuildGeneratedAndKillStatementSets()
{
	dTree<dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();
		point.m_killStmtSet.RemoveAll();
	}

	m_variableDefinitions.RemoveAll();
	dDataFlowPoint::dVariableSet<dCIL::dListNode*> statementUsingReturnVariable;
	for (dCIL::dListNode* ptr = m_function; ptr; ptr = ptr->GetNext()) {

		dTreeAdressStmt& stmt = ptr->GetInfo();	
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_assigment:
			{
				dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(stmt.m_arg0.m_label);
				node->GetInfo().Append(ptr);

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					if (stmt.m_arg1.m_label == m_returnVariableName) {
						statementUsingReturnVariable.Insert(ptr);
					}
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						if (stmt.m_arg2.m_label == m_returnVariableName) {
							statementUsingReturnVariable.Insert(ptr);
						}
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}


			case dTreeAdressStmt::m_load:
			{
				dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(stmt.m_arg0.m_label);
				node->GetInfo().Append(ptr);

				if (stmt.m_arg1.m_label == m_returnVariableName) {
					statementUsingReturnVariable.Insert(ptr);
				}

				if (stmt.m_arg2.m_label == m_returnVariableName) {
					statementUsingReturnVariable.Insert(ptr);
				}
				break;
			}


			case dTreeAdressStmt::m_push:
			{
				if (stmt.m_arg1.m_label == m_returnVariableName) {
					statementUsingReturnVariable.Insert(ptr);
				}
				break;
			}

			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_paramLoad:
			{
				dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(stmt.m_arg0.m_label);
				node->GetInfo().Append(ptr);
				break;
			}


			case dTreeAdressStmt::m_if:
			{
				if (stmt.m_arg0.m_type == dTreeAdressStmt::m_intVar) {
					if (stmt.m_arg0.m_label == m_returnVariableName) {
						statementUsingReturnVariable.Insert(ptr);
					}
				}

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					if (stmt.m_arg1.m_label == m_returnVariableName) {
						statementUsingReturnVariable.Insert(ptr);
					}
				}
				break;
			}


			case dTreeAdressStmt::m_call:
			{
				if (stmt.m_extraInformation == dCIL::m_intRegister) {
					dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(GetReturnVariableName());
					node->GetInfo().Append(ptr);
				} else if (stmt.m_extraInformation == dCIL::m_floatRegister) {
					dAssert (0);
				}
				break;					
			}

			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_store:
			{
				if (stmt.m_arg0.m_label == m_returnVariableName) {
					statementUsingReturnVariable.Insert(ptr);
				}
				break;
			}

			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_argument:
			case dTreeAdressStmt::m_function:
			break;

			default:
				dAssert (0);
		}
	}


	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();

		point.m_generateStmt = false;
		dTreeAdressStmt& stmt = point.m_statement->GetInfo();	
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_paramLoad:
			case dTreeAdressStmt::m_assigment:
			{
				point.m_generateStmt = true;
				dAssert (m_variableDefinitions.Find(stmt.m_arg0.m_label));
				dList<dCIL::dListNode*>& defsList = m_variableDefinitions.Find(stmt.m_arg0.m_label)->GetInfo();
				for (dList<dCIL::dListNode*>::dListNode* defNode = defsList.GetFirst(); defNode; defNode = defNode->GetNext()) {
					dCIL::dListNode* const killStement = defNode->GetInfo();
					if (killStement != point.m_statement) {
						point.m_killStmtSet.Insert(killStement);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_call:
			{
				if (m_returnType != dCIL::m_void) {
					point.m_generateStmt = true;
					dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const retRegisterNode = m_variableDefinitions.Find(m_returnVariableName);
					dList<dCIL::dListNode*>& defsList = retRegisterNode->GetInfo();
					for (dList<dCIL::dListNode*>::dListNode* defNode = defsList.GetFirst(); defNode; defNode = defNode->GetNext()) {
						dCIL::dListNode* const killStement = defNode->GetInfo();
						if (killStement != point.m_statement) {
							point.m_killStmtSet.Insert(killStement);
						}
					}
					dDataFlowPoint::dVariableSet<dCIL::dListNode*>::Iterator iter (statementUsingReturnVariable);
					for (iter.Begin(); iter; iter ++) {
						dCIL::dListNode* const stmtNode = iter.GetKey();
						point.m_killStmtSet.Insert(stmtNode);
					}
				}

				break;
			}

			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_store:
			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_if:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_argument:
				break;

			default:
				dAssert (0);
		}
	}
}

void dDataFlowGraph::CalculateReachingDefinitions()
{
	BuildGeneratedAndKillStatementSets ();

	dTree<dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();
		point.m_reachStmtInputSet.RemoveAll();
		point.m_reachStmtOutputSet.RemoveAll();
	}

	bool someSetChanged = false;
	while (!someSetChanged) {
		someSetChanged = true;
		for (dList<dBasicBlock*>::dListNode* blockNode = m_traversalBlocksOrder.GetLast(); blockNode; blockNode = blockNode->GetPrev()) {
			dBasicBlock* const block = blockNode->GetInfo();

			int test = 1;
			for (dCIL::dListNode* stmtNode = block->m_begin; test; stmtNode = stmtNode->GetNext()) {
				test = (stmtNode != block->m_end);

				dAssert (m_dataFlowGraph.Find(stmtNode));
				dDataFlowPoint& info = m_dataFlowGraph.Find(stmtNode)->GetInfo();
				dDataFlowPoint::dVariableSet<dCIL::dListNode*> oldInput (info.m_reachStmtInputSet);
				dDataFlowPoint::dVariableSet<dCIL::dListNode*> oldOutput (info.m_reachStmtOutputSet);

				info.m_reachStmtInputSet.RemoveAll();
				for (dList<dDataFlowPoint*>::dListNode* predecessorsNode = info.m_predecessors.GetFirst(); predecessorsNode; predecessorsNode = predecessorsNode->GetNext()) {
					dDataFlowPoint* const predecessorsInfo = predecessorsNode->GetInfo();
					info.m_reachStmtInputSet.Union (predecessorsInfo->m_reachStmtOutputSet);
				}

				info.m_reachStmtOutputSet.RemoveAll();
				info.m_reachStmtOutputSet.Difference (info.m_reachStmtInputSet, info.m_killStmtSet);
				//info.m_reachOutputSet.Union (info.m_generateVariable);
				if (info.m_generateStmt) {
					info.m_reachStmtOutputSet.Insert (info.m_statement);
				}

				someSetChanged = (someSetChanged && oldOutput.Compare(info.m_reachStmtOutputSet) && oldInput.Compare(info.m_reachStmtInputSet));
			}
		}
	}


#ifdef _DEBUG
/*
	for (dCIL::dListNode* ptr = m_function; ptr; ptr = ptr->GetNext()) {
		dTreeAdressStmt& stmt = ptr->GetInfo();	
		if ((stmt.m_instruction != dTreeAdressStmt::m_nop) && (stmt.m_instruction != dTreeAdressStmt::m_function)) {
			dTrace (("%d ", stmt.m_debug));
			DTRACE_INTRUCTION(&stmt);
		}
	}

	dTrace (("\n\n"));

	for (dCIL::dListNode* ptr = m_function; ptr; ptr = ptr->GetNext()) {
		dTreeAdressStmt& stmt = ptr->GetInfo();	
		if ((stmt.m_instruction != dTreeAdressStmt::m_nop) && (stmt.m_instruction != dTreeAdressStmt::m_function)) {
			dTrace (("%d ", stmt.m_debug));
			DTRACE_INTRUCTION(&stmt);
			const dDataFlowPoint& point = m_dataFlowGraph.Find(ptr)->GetInfo() ;

			if (point.m_reachStmtsInputSet.GetCount()) {
				dTrace (("\t\t"));
				dDataFlowPoint::dVariableSet<dCIL::dListNode*>::Iterator iter(point.m_reachStmtsInputSet);
				for (iter.Begin(); iter; iter ++) {
					dCIL::dListNode* const stmtNode = iter.GetKey();
					dTreeAdressStmt& stmt1 = stmtNode->GetInfo();
					dTrace (("%d ", stmt1.m_debug));
				}
				dTrace (("\n"));
			}
		}
	}
*/
#endif

}

void dDataFlowGraph::UpdateReachingDefinitions()
{
	// for now just brute force regeneration
	CalculateReachingDefinitions();
}


void dDataFlowGraph::FindNodesInPathway(dCIL::dListNode* const source, dCIL::dListNode* const destination, dTree<int, dCIL::dListNode*>& pathOut) const
{
	m_mark ++;
	dList<dDataFlowPoint*> queue; 
	queue.Append(&m_dataFlowGraph.Find(source)->GetInfo());

	while (queue.GetCount()) {
		dDataFlowPoint* const rootNode = queue.GetFirst()->GetInfo();
		queue.Remove(queue.GetFirst());
		if (rootNode->m_mark != m_mark) {
			rootNode->m_mark = m_mark;
			pathOut.Insert(rootNode->m_statement);

			for (dList<dDataFlowPoint*>::dListNode* successorsNode = rootNode->m_successors.GetFirst(); successorsNode; successorsNode = successorsNode->GetNext()) {
				dDataFlowPoint* const successor = successorsNode->GetInfo();
				if (successor->m_statement != destination) {
					queue.Append(successor);
				}
			}
		}
	}

}


bool dDataFlowGraph::DoStatementAreachesStatementB(dCIL::dListNode* const stmtNodeB, dCIL::dListNode* const stmtNodeA) const
{
	bool canApplyPropagation = false;	
	dDataFlowPoint& info = m_dataFlowGraph.Find(stmtNodeB)->GetInfo();
	dDataFlowPoint::dVariableSet<dCIL::dListNode*>& reachingInputs = info.m_reachStmtInputSet;

	if (reachingInputs.Find (stmtNodeA)) {
		canApplyPropagation = true;
		const dTreeAdressStmt& constStmt = stmtNodeA->GetInfo();
		dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const definitions = m_variableDefinitions.Find(constStmt.m_arg0.m_label);
		dAssert (definitions);

		dList<dCIL::dListNode*>& defintionList = definitions->GetInfo();
		for (dList<dCIL::dListNode*>::dListNode* otherStmtNode = defintionList.GetFirst(); otherStmtNode; otherStmtNode = otherStmtNode->GetNext()){
			dCIL::dListNode* const duplicateDefinition = otherStmtNode->GetInfo();
			if (duplicateDefinition != stmtNodeA) {
				if (reachingInputs.Find(duplicateDefinition)) {
					return false;
				}
			}
		}

		if ((constStmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) || (constStmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar)) {
			dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const definedStatements = m_variableDefinitions.Find(constStmt.m_arg1.m_label);
			if (definedStatements) {
				dTree<int, dCIL::dListNode*> path;
				FindNodesInPathway(stmtNodeA, stmtNodeB, path);
				dList<dCIL::dListNode*>& statementList = definedStatements->GetInfo();
				for (dList<dCIL::dListNode*>::dListNode* ptr = statementList.GetFirst(); ptr; ptr = ptr->GetNext()) {
					dCIL::dListNode* const stmt = ptr->GetInfo();
					if (path.Find(stmt)) {
						return false;
					}
				}

				
				if ((constStmt.m_instruction == dTreeAdressStmt::m_load) || ((constStmt.m_operator != dTreeAdressStmt::m_nothing) && ((constStmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) || (constStmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar)))) {
					dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const definedStatements = m_variableDefinitions.Find(constStmt.m_arg2.m_label);
					if (definedStatements) {
						dList<dCIL::dListNode*>& statementList = definedStatements->GetInfo();
						for (dList<dCIL::dListNode*>::dListNode* ptr = statementList.GetFirst(); ptr; ptr = ptr->GetNext()) {
							dCIL::dListNode* const stmt = ptr->GetInfo();
							if (path.Find(stmt)) {
								return false;
							}
						}
					}
				}
			}
		}
	}

	return canApplyPropagation;
}


void dDataFlowGraph::BuildGeneratedAndKillVariableSets()
{
	dTree<dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();
		point.m_generatedVariableSet.RemoveAll();
	}

	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();
		point.m_killVariable.Empty();
		dTreeAdressStmt& stmt = point.m_statement->GetInfo();	

		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_paramLoad:
			{
				point.m_killVariable = stmt.m_arg0.m_label;
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				point.m_killVariable = stmt.m_arg0.m_label;

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						point.m_generatedVariableSet.Insert(stmt.m_arg2.m_label);
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_alloc:
			{
				point.m_killVariable = stmt.m_arg0.m_label;
				point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}

			case dTreeAdressStmt::m_free:
			{
				point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}


			case dTreeAdressStmt::m_call:
			{
				if (m_returnType != dCIL::m_void) {
					point.m_killVariable = m_returnVariableName;
				}
				break;
			}

			case dTreeAdressStmt::m_ret:
			{
				if (m_returnType != dCIL::m_void) {
					point.m_generatedVariableSet.Insert(m_returnVariableName);
				}
				break;
			}


			case dTreeAdressStmt::m_load:
			{
				point.m_killVariable = stmt.m_arg0.m_label;
				point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				point.m_generatedVariableSet.Insert(stmt.m_arg2.m_label);
				break;
			}

			case dTreeAdressStmt::m_store:
			{
				point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				point.m_generatedVariableSet.Insert(stmt.m_arg2.m_label);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				if (stmt.m_arg0.m_type == dTreeAdressStmt::m_intVar) {
					point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				}

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				}
				break;
			}

			case dTreeAdressStmt::m_push:
			{
				point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}


//			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
				break;

			default:
				dAssert (0);
		}
	}
}



void dDataFlowGraph::CalculateLiveInputLiveOutput ()
{
	BuildGeneratedAndKillVariableSets();

	dTree<dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();
		point.m_liveInputSet.RemoveAll();
		point.m_liveOutputSet.RemoveAll();
	}

	bool someSetChanged = false;
	while (!someSetChanged) {
		someSetChanged = true;
		for (dList<dBasicBlock*>::dListNode* blockNode = m_traversalBlocksOrder.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {

			dBasicBlock* const block = blockNode->GetInfo();

			int test = 1;
			for (dCIL::dListNode* stmtNode = block->m_end; test; stmtNode = stmtNode->GetPrev()) {
				test = (stmtNode != block->m_begin);

				dAssert (m_dataFlowGraph.Find(stmtNode));
				dDataFlowPoint& info = m_dataFlowGraph.Find(stmtNode)->GetInfo();
				dDataFlowPoint::dVariableSet<dString> oldInput (info.m_liveInputSet);
				dDataFlowPoint::dVariableSet<dString> oldOutput (info.m_liveOutputSet);

				info.m_liveInputSet.RemoveAll();
				info.m_liveInputSet.Union(info.m_liveOutputSet);
				if (info.m_killVariable.Size()) {
					info.m_liveInputSet.Remove(info.m_killVariable);
				}
				info.m_liveInputSet.Union(info.m_generatedVariableSet);

				info.m_liveOutputSet.RemoveAll();
				for (dList<dDataFlowPoint*>::dListNode* successorNode = info.m_successors.GetFirst(); successorNode; successorNode = successorNode->GetNext()) {
					dDataFlowPoint* const successorInfo = successorNode->GetInfo();
					info.m_liveOutputSet.Union (successorInfo->m_liveInputSet);
				}
				someSetChanged = (someSetChanged && oldOutput.Compare(info.m_liveOutputSet) && oldInput.Compare(info.m_liveInputSet));
			}
		}
	}
}

void dDataFlowGraph::UpdateLiveInputLiveOutput()
{
	CalculateLiveInputLiveOutput ();
}

int dDataFlowGraph::EvaluateBinaryExpression (const dString& arg1, dTreeAdressStmt::dOperator operation, const dString& arg2) const
{
	int operando = 0;
	int operando1 = arg1.ToInteger();
	int operando2 = arg2.ToInteger();

	switch (operation) 
	{
		case dTreeAdressStmt::m_add:
			operando = operando1 + operando2;
			break;

		case dTreeAdressStmt::m_greather:
			operando = operando1 > operando2;
			break;

		case dTreeAdressStmt::m_less:
			operando = operando1 < operando2;
			break;

		case dTreeAdressStmt::m_sub:
			operando = operando1 - operando2;
			break;

		case dTreeAdressStmt::m_mul:
			operando = operando1 * operando2;
			break;

		case dTreeAdressStmt::m_div:
			operando = operando1 / operando2;
			break;

		case dTreeAdressStmt::m_mod:
			operando = operando1 % operando2;
			break;

		case dTreeAdressStmt::m_equal:
			operando = (operando1 = operando2);
			break;

		case dTreeAdressStmt::m_identical:
			operando = (operando1 == operando2);
			break;

		case dTreeAdressStmt::m_different:
			operando = (operando1 != operando2);
			break;

		case dTreeAdressStmt::m_lessEqual:
			operando = (operando1 <= operando2);
			break;

		case dTreeAdressStmt::m_greatherEqual:
			operando = (operando1 >= operando2);
			break;

		case dTreeAdressStmt::m_operatorsCount:

		default:
			dAssert (0);
	}
	return operando;
}




bool dDataFlowGraph::ApplyConstantPropagation()
{
	bool ret = false;
	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dTreeAdressStmt& stmt = stmtNode->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_assigment) && (stmt.m_operator == dTreeAdressStmt::m_nothing) && (stmt.m_arg1.m_type == dTreeAdressStmt::m_intConst)) {
			for (dCIL::dListNode* stmtNode1 = stmtNode->GetNext(); stmtNode1; stmtNode1 = stmtNode1->GetNext()) {
				dTreeAdressStmt& stmt1 = stmtNode1->GetInfo();
				switch (stmt1.m_instruction)
				{
					case dTreeAdressStmt::m_assigment:
					{
						if (stmt1.m_operator == dTreeAdressStmt::m_nothing) {
							if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg1 = stmt.m_arg1;
								UpdateReachingDefinitions();
							}
						} else if (m_cil->m_commutativeOperator[stmt1.m_operator]) {
							
							if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
								dAssert (0);
								ret = true;
								stmt1.m_arg1 = stmt1.m_arg2;
								stmt1.m_arg2 = stmt.m_arg1;
								if ((stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
									stmt1.m_arg1 = stmt.m_arg1;
								}
								UpdateReachingDefinitions();
							} else if ((stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg2 = stmt.m_arg1;
								UpdateReachingDefinitions();
							}
							
						} else {
							if ((stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg2 = stmt.m_arg1;
								UpdateReachingDefinitions();
							}

							if ((stmt1.m_arg2.m_type == dTreeAdressStmt::m_intConst) && (stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg1 = stmt.m_arg1;
								UpdateReachingDefinitions();
							}
						}

/*
						if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)){
							if ((stmt1.m_operator != dTreeAdressStmt::m_nothing) && m_cil->m_commutativeOperator[stmt1.m_operator]) {
							} else {
							ret = true;
							stmt1.m_arg1 = stmt.m_arg1;
						}
						if ((stmt1.m_operator != dTreeAdressStmt::m_nothing) && (stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
							ret = true;
							stmt1.m_arg2 = stmt.m_arg1;
						}
*/
						break;
					}

					case dTreeAdressStmt::m_if:
					{
						if ((stmt1.m_arg0.m_label == stmt.m_arg0.m_label) && (stmt1.m_arg1.m_type == dTreeAdressStmt::m_intConst) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
							ret = true;
							stmt1.m_arg0 = stmt.m_arg1;
							UpdateReachingDefinitions();
						}

/*
						if ((stmt1.m_arg0.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
							ret = true;
							stmt1.m_arg0 = stmt.m_arg1;
						}
						if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
							ret = true;
							stmt1.m_arg1 = stmt.m_arg1;
						}
*/
						break;
					}
				}
			}
		}
	}

	return ret;
}


bool dDataFlowGraph::ApplyIfStatementsSimplification()
{
	bool ret = false;

	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dTreeAdressStmt& stmt = stmtNode->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_if) && (stmt.m_operator == dTreeAdressStmt::m_identical) || (stmt.m_operator == dTreeAdressStmt::m_different)) {
			dCIL::dListNode* const prevStmtNode = stmtNode->GetPrev();
			dTreeAdressStmt& stmtA = prevStmtNode->GetInfo();
			if ((stmt.m_arg1.m_label == stmtA.m_arg0.m_label) && (stmtA.m_instruction == dTreeAdressStmt::m_assigment)  &&  (stmtA.m_operator == dTreeAdressStmt::m_nothing) && (stmtA.m_arg1.m_type == dTreeAdressStmt::m_intConst)) {
				dCIL::dListNode* const prevPrevStmtNode = prevStmtNode->GetPrev();
				dTreeAdressStmt& stmtB = prevPrevStmtNode->GetInfo();
				if ((stmt.m_arg0.m_label == stmtB.m_arg0.m_label) && (stmtB.m_instruction == dTreeAdressStmt::m_assigment)  && (m_cil->m_conditionals[stmtB.m_operator]) && (stmtB.m_arg2.m_type != dTreeAdressStmt::m_intConst)) {
					stmt.m_arg0 = stmtB.m_arg1;
					stmt.m_arg1 = stmtB.m_arg2;
					if (stmt.m_operator == dTreeAdressStmt::m_identical) {
						stmt.m_operator = m_cil->m_operatorComplement[stmtB.m_operator];
					} else {
						stmt.m_operator = stmtB.m_operator;
					}
					ret = true;
				}
			}
		}
	}
	return ret;
}


bool dDataFlowGraph::ApplyCopyPropagation()
{
	bool ret = false;

	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dTreeAdressStmt& stmt = stmtNode->GetInfo();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_assigment:
			{
				if ((stmt.m_operator == dTreeAdressStmt::m_nothing) && (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar)) {
					for (dCIL::dListNode* stmtNode1 = stmtNode->GetNext(); stmtNode1; stmtNode1 = stmtNode1->GetNext()) {
						dTreeAdressStmt& stmt1 = stmtNode1->GetInfo();
						switch (stmt1.m_instruction)
						{
							case dTreeAdressStmt::m_assigment:
							{
								if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)){
									ret = true;
									stmt1.m_arg1 = stmt.m_arg1;
									UpdateReachingDefinitions();
								}
								if ((stmt1.m_operator != dTreeAdressStmt::m_nothing) && (stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
									ret = true;
									stmt1.m_arg2 = stmt.m_arg1;
									UpdateReachingDefinitions();
								}
								break;
							}

							case dTreeAdressStmt::m_load:
							{
								if ((stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
									ret = true;		
									stmt1.m_arg2 = stmt.m_arg1;
									UpdateReachingDefinitions();
								} else if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
									ret = true;		
									stmt1.m_arg1 = stmt.m_arg1;
									UpdateReachingDefinitions();
								}
								break;
							}

							case dTreeAdressStmt::m_store:
							{
								if ((stmt1.m_arg0.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
									ret = true;
									stmt1.m_arg0 = stmt.m_arg1;
									UpdateReachingDefinitions();
								} else if ((stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
									ret = true;		
									stmt1.m_arg2 = stmt.m_arg1;
									UpdateReachingDefinitions();
								} else if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
									ret = true;		
									stmt1.m_arg1 = stmt.m_arg1;
									UpdateReachingDefinitions();
								}
								break;
							}

							case dTreeAdressStmt::m_if:
							{
								if ((stmt1.m_arg0.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)){
									ret = true;
									stmt1.m_arg0 = stmt.m_arg1;
									UpdateReachingDefinitions();
								}
								if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
									ret = true;
									stmt1.m_arg1 = stmt.m_arg1;
									UpdateReachingDefinitions();
								}
								break;
							}
						}
					}
				}
				break;
			}

			case dTreeAdressStmt::m_load:
			{
				#ifdef D_OPTIMIZE_REDUNDNACE_COPIES_ON_ARRAYS
					for (dCIL::dListNode* stmtNode1 = stmtNode->GetNext(); stmtNode1; stmtNode1 = stmtNode1->GetNext()) {
						dTreeAdressStmt& stmt1 = stmtNode1->GetInfo();
						switch (stmt1.m_instruction)
						{
							case dTreeAdressStmt::m_load:
							{
								if ((stmt1.m_arg1.m_label == stmt.m_arg1.m_label) && (stmt1.m_arg2.m_label == stmt.m_arg2.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)){
									ret = true;
									stmt1.m_instruction = dTreeAdressStmt::m_assigment;
									stmt1.m_operator = dTreeAdressStmt::m_nothing;
									stmt1.m_arg1 = stmt.m_arg0;
									stmt1.m_arg2.m_label = "";
									stmt1.m_extraInformation = 0;
									UpdateReachingDefinitions();
								}
								break;
							}
						}
					}
				#endif
				break;
			}
		}
	}

	return ret;
}


bool dDataFlowGraph::ApplySubExpresionToCopyPropagation()
{
	bool ret = false;
	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dTreeAdressStmt& stmt = stmtNode->GetInfo();
		if (stmt.m_instruction == dTreeAdressStmt::m_assigment) {
			if ((stmt.m_operator != dTreeAdressStmt::m_nothing) && (stmt.m_arg0.m_label != stmt.m_arg1.m_label) && (stmt.m_arg0.m_label != stmt.m_arg2.m_label)){
				for (dCIL::dListNode* stmtNode1 = stmtNode->GetNext(); stmtNode1; stmtNode1 = stmtNode1->GetNext()) {
					dTreeAdressStmt& stmt1 = stmtNode1->GetInfo();
					switch (stmt1.m_instruction) 
					{
						case dTreeAdressStmt::m_assigment:
						{
							if ((stmt1.m_operator == dTreeAdressStmt::m_nothing) && (stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode) ){
								ret = true;
								stmt1.m_operator = stmt.m_operator;
								stmt1.m_arg1 = stmt.m_arg1;
								stmt1.m_arg2 = stmt.m_arg2;
								UpdateReachingDefinitions();
							}
							break;
						}

						case dTreeAdressStmt::m_if:
						{
							if (m_cil->m_conditionals[stmt.m_operator] && (stmt1.m_arg1.m_label == "0") && (stmt1.m_arg0.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
								switch (stmt1.m_operator) 
								{
									case dTreeAdressStmt::m_identical:
									{
										ret = true;
										stmt1.m_operator = m_cil->m_operatorComplement[stmt.m_operator];
										stmt1.m_arg0 = stmt.m_arg1;
										stmt1.m_arg1 = stmt.m_arg2;
										UpdateReachingDefinitions();
										break;
									}

									case dTreeAdressStmt::m_different:
									{
										ret = true;
										stmt1.m_operator = stmt.m_operator;
										stmt1.m_arg0 = stmt.m_arg1;
										stmt1.m_arg1 = stmt.m_arg2;
										UpdateReachingDefinitions();
										break;
									}
								}
							}
							break;
						}
					}
				}
			} 
		}
	}
	return ret;
}



bool dDataFlowGraph::ApplyCommonSubExpresion()
{
	bool ret = false;
	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dTreeAdressStmt& stmt = stmtNode->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_assigment) && (stmt.m_operator != dTreeAdressStmt::m_nothing)) {
			for (dCIL::dListNode* stmtNode1 = stmtNode->GetNext(); stmtNode1; stmtNode1 = stmtNode1->GetNext()) {
				dTreeAdressStmt& stmt1 = stmtNode1->GetInfo();
				if ((stmt1.m_instruction == dTreeAdressStmt::m_assigment) && (stmt1.m_operator == stmt.m_operator)) {
					if ((stmt.m_arg1.m_label == stmt1.m_arg1.m_label) && (stmt.m_arg2.m_label == stmt1.m_arg2.m_label)) {
						if (DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
							ret = true;
							stmt1.m_operator = stmt.m_nothing;
							stmt1.m_arg1 = stmt.m_arg0;
							stmt1.m_arg2.m_label = "";
							UpdateReachingDefinitions();
						}
					}
				}
			}
		}
	}

	return ret;
}


bool dDataFlowGraph::ApplyConstantFolding()
{
	bool ret = false;
	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dTreeAdressStmt& stmt = stmtNode->GetInfo();

		switch  (stmt.m_instruction) 
		{
			case dTreeAdressStmt::m_if:
			{
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intConst) {
					ret = true;
					int val = EvaluateBinaryExpression (stmt.m_arg0.m_label, stmt.m_operator, stmt.m_arg1.m_label);
					if (val) {
						stmt.m_instruction = dTreeAdressStmt::m_goto;
						stmt.m_arg0.m_label = stmt.m_arg2.m_label;
					} else {
						stmt.m_instruction = dTreeAdressStmt::m_nop;
					}
				} else if ((stmt.m_arg0.m_type == dTreeAdressStmt::m_intVar) && (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar)) {
					dList<dCIL::dListNode*>& argListNodeA = m_variableDefinitions.Find (stmt.m_arg0.m_label)->GetInfo();
					dList<dCIL::dListNode*>& argListNodeB = m_variableDefinitions.Find (stmt.m_arg1.m_label)->GetInfo();
					if ((argListNodeA.GetCount() == 1) && (argListNodeB.GetCount() == 1)) {
						dTreeAdressStmt& argStmtA = argListNodeA.GetFirst()->GetInfo()->GetInfo();
						dTreeAdressStmt& argStmtB = argListNodeB.GetFirst()->GetInfo()->GetInfo();
						if ((argStmtA.m_arg1.m_type == dTreeAdressStmt::m_intConst) && (argStmtB.m_arg1.m_type == dTreeAdressStmt::m_intConst)) {
							ret = true;
							stmt.m_arg0 = argStmtA.m_arg1;
							stmt.m_arg1 = argStmtB.m_arg1;
							int val = EvaluateBinaryExpression (stmt.m_arg0.m_label, stmt.m_operator, stmt.m_arg1.m_label);
							if (val) {
								stmt.m_instruction = dTreeAdressStmt::m_goto;
								stmt.m_arg0.m_label = stmt.m_arg2.m_label;
							} else {
								stmt.m_instruction = dTreeAdressStmt::m_nop;
							}
							UpdateReachingDefinitions();
//m_cil->Trace();
						}
					}
				}
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (((stmt.m_arg1.m_type == dTreeAdressStmt::m_intConst) || (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatConst)) &&
						((stmt.m_arg2.m_type == dTreeAdressStmt::m_intConst) || (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatConst))) {
						ret = true;
						if ((stmt.m_arg1.m_type == dTreeAdressStmt::m_intConst) && (stmt.m_arg1.m_type == dTreeAdressStmt::m_intConst)) {
							int arg0 = EvaluateBinaryExpression (stmt.m_arg1.m_label, stmt.m_operator, stmt.m_arg2.m_label);
							stmt.m_operator = dTreeAdressStmt::m_nothing;
							stmt.m_arg1.m_label = dString(arg0);
							stmt.m_arg2.m_label = dString ("");

						} else if ((stmt.m_arg1.m_type == dTreeAdressStmt::m_floatConst) && (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatConst)) {
							dAssert (0);
						} else {
							dAssert (0);
						}
					}
				}
				break;
			}
		}
	}
	return ret;
}

bool dDataFlowGraph::ApplyInstructionSematicOrdering()
{
	bool ret = false;
	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dTreeAdressStmt& stmt = stmtNode->GetInfo();
		
		switch (stmt.m_instruction) 
		{
			case dTreeAdressStmt::m_assigment:
			{
//				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
//					if ((stmt.m_arg1.m_type == dTreeAdressStmt::m_intConst) && (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar)) {			
//						dAssert (0);
//					} else if (m_cil->m_commutativeOperator[stmt.m_operator] && (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) && (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar)) {
//						dAssert (0);
//					}
//				}
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				if ((stmt.m_arg0.m_type == dTreeAdressStmt::m_intConst) && (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar)) {
					stmt.m_operator = m_cil->m_operatorComplement[stmt.m_operator];
					dTreeAdressStmt::dArg arg (stmt.m_arg0);
					stmt.m_arg0 = stmt.m_arg1;
					stmt.m_arg1 = arg;
				}
			}
		}
	}
	return ret;
}



bool dDataFlowGraph::ApplyRemoveDeadCode()
{
	CalculateLiveInputLiveOutput ();

	bool ret = false;
	dCIL::dListNode* nextStmtNode;
	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = nextStmtNode) {
		dTreeAdressStmt& stmt = stmtNode->GetInfo();
		nextStmtNode = stmtNode->GetNext();
		switch (stmt.m_instruction) 
		{
			case dTreeAdressStmt::m_assigment:
			{
				dDataFlowPoint& info = m_dataFlowGraph.Find(stmtNode)->GetInfo();
				dDataFlowPoint::dVariableSet<dString>& liveOut = info.m_liveOutputSet;
				if (!liveOut.Find (stmt.m_arg0.m_label)) {
					ret = true;
					stmt.m_instruction = dTreeAdressStmt::m_nop;
					UpdateLiveInputLiveOutput();
				} else if ((stmt.m_operator == dTreeAdressStmt::m_nothing) && (stmt.m_arg0.m_label == stmt.m_arg1.m_label)){
					ret = true;
					stmt.m_instruction = dTreeAdressStmt::m_nop;
					UpdateLiveInputLiveOutput();
				}
				break;
			}

			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_paramLoad:
			{
				dDataFlowPoint& info = m_dataFlowGraph.Find(stmtNode)->GetInfo();
				dDataFlowPoint::dVariableSet<dString>& liveOut = info.m_liveOutputSet;
				if (!liveOut.Find (stmt.m_arg0.m_label)) {
					ret = true;
					stmt.m_instruction = dTreeAdressStmt::m_nop;
					UpdateLiveInputLiveOutput();
				}
				break;
			}
		}
	}
	return ret;
}


dDataFlowGraph::dRegisterInterferenceGraph::dRegisterInterferenceGraph (dDataFlowGraph* const flowGraph, int registerCount)
	:dTree<dRegisterInterferenceNode, dString>()
	,m_flowGraph(flowGraph)
	,m_registerCount(registerCount)
{
	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
	while (m_flowGraph->ApplyRemoveDeadCode());

	dTree<dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_flowGraph->m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();

		dDataFlowPoint::dVariableSet<dString>::Iterator definedIter (point.m_generatedVariableSet);
		for (definedIter.Begin(); definedIter; definedIter ++) {
			const dString& variable = definedIter.GetKey();
			dTreeNode* interferanceGraphNode = Find(variable);
			if (!interferanceGraphNode) {
				interferanceGraphNode = Insert(variable);
				interferanceGraphNode->GetInfo().m_name = variable;
				dTrace (("%s\n", variable.GetStr()));
			}
		}
	}


	// pre-color some special nodes
	dTreeNode* const returnRegNode = Find(GetReturnVariableName());
	if (returnRegNode) {
		dRegisterInterferenceNode& returnReginster = returnRegNode->GetInfo();
		returnReginster.m_registerIndex = D_RETURN_REGISTER_INDEX;
		m_flowGraph->m_returnVariableName = m_flowGraph->GetRegisterName (*this, returnReginster.m_name);
	}

	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& info = iter.GetNode()->GetInfo();
		if (info.m_killVariable.Size()) {
			const dString& variableA = info.m_killVariable;

			dTreeNode* const nodeA = Find(variableA);
			dAssert (nodeA);
			dRegisterInterferenceNode& inteferanceNodeA = nodeA->GetInfo();
			dDataFlowPoint::dVariableSet<dString>::Iterator aliveIter (info.m_liveOutputSet);

			for (aliveIter.Begin(); aliveIter; aliveIter ++) {
				const dString& variableB = aliveIter.GetKey();

				if (variableA != variableB) {
					dTreeNode* const nodeB = Find(variableB);
					dAssert (nodeB);
					dRegisterInterferenceNode& inteferanceNodeB = nodeB->GetInfo();

					bool hasEdge = false;
					for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = inteferanceNodeA.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
						if (ptr->GetInfo().m_targetNode == nodeB) {
							hasEdge = true;
							break;
						}
					}
					if (!hasEdge) {
						dList<dRegisterInterferenceNodeEdge>::dListNode* const entryA = inteferanceNodeA.m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeB));
						dList<dRegisterInterferenceNodeEdge>::dListNode* const entryB = inteferanceNodeB.m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeA));
						entryA->GetInfo().m_twin = &entryB->GetInfo();
						entryB->GetInfo().m_twin = &entryA->GetInfo();
					}
				}
			}
		}
	}

	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& info = iter.GetNode()->GetInfo();
		dTreeAdressStmt& stmt = info.m_statement->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_assigment) && (stmt.m_operator == dTreeAdressStmt::m_nothing)) {
			const dString& variableA = stmt.m_arg0.m_label;
			const dString& variableB = stmt.m_arg1.m_label;
			dTreeNode* const nodeA = Find(variableA);
			dTreeNode* const nodeB = Find(variableB);
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = nodeA->GetInfo().m_interferanceEdge.GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				if (edge.m_targetNode == nodeB) {
					edge.m_isMov = true;
					edge.m_twin->m_isMov = true;
					break;
				}
			}
		}
	}

	int registersUsed = ColorGraph (registerCount);
	m_flowGraph->AllocateRegisters(*this);

m_flowGraph->m_cil->Trace();
	if (registersUsed > registerCount) {
		// we have some spill fix the program.
		AllocatedSpilledRegister(registersUsed, registerCount);
	}

	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
m_flowGraph->m_cil->Trace();

	for (bool optimized = true; optimized;) {
		optimized = false;
		m_flowGraph->UpdateReachingDefinitions();
		optimized |= m_flowGraph->ApplyCopyPropagation();
m_flowGraph->m_cil->Trace();
		optimized |= m_flowGraph->ApplyRemoveDeadCode();
m_flowGraph->m_cil->Trace();
	}
}

void dDataFlowGraph::dRegisterInterferenceGraph::SortRegistersByFrequency(int totalRegisters, int registerCount)
{
	memset (m_registenFrequency, 0, sizeof (m_registenFrequency)) ;

	for (int i = 0; i < totalRegisters; i ++) {
		m_registenFrequency[i][0] = 0;
		m_registenFrequency[i][1] = i;
	}

	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = node->GetNext()) {
		const dTreeAdressStmt& stmt = node->GetInfo();
//stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_assigment:
			{
				m_registenFrequency[RegisterToIndex (stmt.m_arg0.m_label)][0] += 1;

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					m_registenFrequency[RegisterToIndex (stmt.m_arg1.m_label)][0] += 1;
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						m_registenFrequency[RegisterToIndex (stmt.m_arg2.m_label)][0] += 1;
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_store:
			{
				m_registenFrequency[RegisterToIndex (stmt.m_arg0.m_label)][0] += 1;
				m_registenFrequency[RegisterToIndex (stmt.m_arg1.m_label)][0] += 1;
				m_registenFrequency[RegisterToIndex (stmt.m_arg2.m_label)][0] += 1;
				break;
			}

			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_paramLoad:
			{
				m_registenFrequency[RegisterToIndex (stmt.m_arg0.m_label)][0] += 1;
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				if (stmt.m_arg0.m_type == dTreeAdressStmt::m_intVar) {
					m_registenFrequency[RegisterToIndex (stmt.m_arg0.m_label)][0] += 1;
				}

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					m_registenFrequency[RegisterToIndex (stmt.m_arg1.m_label)][0] += 1;
				}
				break;
			}

/*
		case dTreeAdressStmt::m_alloc:
		{
			point.m_killVariable = stmt.m_arg0.m_label;
			point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
			break;
		}
		case dTreeAdressStmt::m_free:
		{
			point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
			break;
		}
*/
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
			break;
	
			default:
				dAssert (0);
		}
	}

	SortRegisters(totalRegisters);


	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();
		switch (stmt.m_instruction)
		{

			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_paramLoad:
			{
				RemapRegister(stmt.m_arg0, totalRegisters);
				break;
			}

			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_store:
			{
				RemapRegister(stmt.m_arg0, totalRegisters);
				RemapRegister(stmt.m_arg1, totalRegisters);
				RemapRegister(stmt.m_arg2, totalRegisters);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				RemapRegister(stmt.m_arg0, totalRegisters);

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					RemapRegister(stmt.m_arg1, totalRegisters);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						RemapRegister(stmt.m_arg2, totalRegisters);
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				if (stmt.m_arg0.m_type == dTreeAdressStmt::m_intVar) {
					RemapRegister(stmt.m_arg0, totalRegisters);
				}

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					RemapRegister(stmt.m_arg1, totalRegisters);
				}
				break;
			}
/*
			case dTreeAdressStmt::m_alloc:
			{
				point.m_killVariable = stmt.m_arg0.m_label;
				point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}
			case dTreeAdressStmt::m_free:
			{
				point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}
*/
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
			break;
	
			default:
				dAssert (0);
		}
	}
}

void dDataFlowGraph::dRegisterInterferenceGraph::AllocatedSpilledRegister(int totalRegisters, int registerCount)
{
	SortRegistersByFrequency(totalRegisters, registerCount);
m_flowGraph->m_cil->Trace();

	int index0 = registerCount - 1;
	if (m_flowGraph->m_returnType != dCIL::m_void) {
		index0 = (index0 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
	}
	int index1 = index0 - 1;
	if (m_flowGraph->m_returnType != dCIL::m_void) {
		index1 = (index1 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
	}
	int index2 = index1 - 1;
	if (m_flowGraph->m_returnType != dCIL::m_void) {
		index2 = (index2 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
	}

	m_reg0 = dString(IndexToRegister(index0));
	m_reg1 = dString (IndexToRegister(index1));
	m_reg2 = dString (IndexToRegister(index2));

	m_local0 = dString (dString("_local") + dString(index0));
	m_local1 = dString (dString("_local") + dString(index1));
	m_local2 = dString (dString("_local") + dString(index2));

	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = nextNode) {
		nextNode = node->GetNext();
		dTreeAdressStmt& stmt = node->GetInfo();
//stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_paramLoad:
			{
				SaveRegisterToTemp (node, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				SaveRegisterToTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					LoadRegisterFromTemp (node, stmt.m_arg2);
				}
				break;
			}

			case dTreeAdressStmt::m_store:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);
				LoadRegisterFromTemp (node, stmt.m_arg2);
				break;
			}

			case dTreeAdressStmt::m_load:
			{
				SaveRegisterToTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);
				LoadRegisterFromTemp (node, stmt.m_arg2);
				break;
			}

			case dTreeAdressStmt::m_push:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);
				break;
			}

/*
			case dTreeAdressStmt::m_alloc:
			{
				point.m_killVariable = stmt.m_arg0.m_label;
				point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}
			case dTreeAdressStmt::m_free:
			{
				point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}
*/
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
			break;
	
			default:
				dAssert (0);
		}
	}

m_flowGraph->m_cil->Trace();


	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = nextNode) {
		nextNode = node->GetNext();
		dTreeAdressStmt& stmt = node->GetInfo();
//stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_paramLoad:
			{
				SaveSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				SaveSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1, registerCount);
				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					LoadSpillRegister (node, m_reg2, stmt.m_arg2, registerCount);
				}
				break;
			}

			case dTreeAdressStmt::m_store:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1, registerCount);
				LoadSpillRegister (node, m_reg2, stmt.m_arg2, registerCount);
				break;
			}

			case dTreeAdressStmt::m_load:
			{
				SaveSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1, registerCount);
				LoadSpillRegister (node, m_reg2, stmt.m_arg2, registerCount);
				break;
			}

			case dTreeAdressStmt::m_push:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1, registerCount);
				break;
			}

			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_enter:
			{
				stmt.m_extraInformation = m_flowGraph->m_registersUsedMask & ((1 << registerCount) - 1);
				break;
			}

/*
			case dTreeAdressStmt::m_alloc:
			{
				point.m_killVariable = stmt.m_arg0.m_label;
				point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}
			case dTreeAdressStmt::m_free:
			{
				point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}
*/

			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
			break;
	
			default:
				dAssert (0);
		}
	}

m_flowGraph->m_cil->Trace();
}



void dDataFlowGraph::dRegisterInterferenceGraph::SaveSpillRegister(dCIL::dListNode* const node, const dString& alliasRegister, dTreeAdressStmt::dArg& argument, int registerCount)
{
	int index = RegisterToIndex (argument.m_label);
	if (index >= registerCount) {
		argument.m_label = alliasRegister;
		SaveRegisterToTemp (node, alliasRegister, IndexToLocal(index));
	}
}

void dDataFlowGraph::dRegisterInterferenceGraph::LoadSpillRegister(dCIL::dListNode* const node, const dString& alliasRegister, dTreeAdressStmt::dArg& argument, int registerCount)
{
	int index = RegisterToIndex (argument.m_label);
	if (index >= registerCount) {
		LoadRegisterFromTemp (node, alliasRegister, IndexToLocal(index));
		argument.m_label = alliasRegister;
	}
}


void dDataFlowGraph::dRegisterInterferenceGraph::SaveRegisterToTemp(dCIL::dListNode* const node, const dString& reg, const dString& local)
{
	dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
	m_flowGraph->m_cil->InsertAfter (node, newNode);
/*
	dTreeAdressStmt& stmt = newNode->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_storeBase;
	stmt.m_operator = dTreeAdressStmt::m_nothing;
	stmt.m_arg0.m_label = local;
	stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
	stmt.m_arg2.m_label = reg;
	stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;
*/

	dTreeAdressStmt& stmt = newNode->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_assigment;
	stmt.m_operator = dTreeAdressStmt::m_nothing;
	stmt.m_arg0.m_label = local;
	stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
	stmt.m_arg1.m_label = reg;
	stmt.m_arg1.m_type = dTreeAdressStmt::m_intVar;
}


void dDataFlowGraph::dRegisterInterferenceGraph::SaveRegisterToTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument)
{
	if (argument.m_label == m_reg0) {
		SaveRegisterToTemp(node, m_reg0, m_local0);
	} else if (argument.m_label == m_reg1) {
		SaveRegisterToTemp(node, m_reg1, m_local1);
	} else if (argument.m_label == m_reg2) {
		SaveRegisterToTemp(node, m_reg2, m_local2);
	}
}

void dDataFlowGraph::dRegisterInterferenceGraph::LoadRegisterFromTemp(dCIL::dListNode* const node, const dString& reg, const dString& local)
{
	dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
	m_flowGraph->m_cil->InsertAfter (node->GetPrev(), newNode);
/*
	dTreeAdressStmt& stmt = newNode->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_loadBase;
	stmt.m_arg0.m_label = reg;
	stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
	stmt.m_arg2.m_label = local;
	stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;
*/

	dTreeAdressStmt& stmt = newNode->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_assigment;
	stmt.m_operator = dTreeAdressStmt::m_nothing;
	stmt.m_arg0.m_label = reg;
	stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
	stmt.m_arg1.m_label = local;
	stmt.m_arg1.m_type = dTreeAdressStmt::m_intVar;
}

void dDataFlowGraph::dRegisterInterferenceGraph::LoadRegisterFromTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument)
{
	if (argument.m_label == m_reg0) {
		LoadRegisterFromTemp(node, m_reg0, m_local0);
	} else if (argument.m_label == m_reg1) {
		LoadRegisterFromTemp(node, m_reg1, m_local1);
	} else if (argument.m_label == m_reg2) {
		LoadRegisterFromTemp(node, m_reg2, m_local2);
	}
}

int dDataFlowGraph::dRegisterInterferenceGraph::Compare (const void* p1, const void* p2)
{
	int* const reg0 = (int*) p1;
	int* const reg1 = (int*) p2;

	if (reg0[0] > reg1[0]) {
		return -1;
	} else if (reg0[0] < reg1[0]) {
		return 1; 
	} 
	return 0;
}


int dDataFlowGraph::dRegisterInterferenceGraph::FindRegister(int regIndex, int totalRegisters) const
{
	for (int i = 0; i < totalRegisters; i ++) {
		if (m_registenFrequency[i][1] == regIndex) {
			return i;
		}
	}
	dAssert (0);
	return -1;
}


void dDataFlowGraph::dRegisterInterferenceGraph::RemapRegister(dTreeAdressStmt::dArg& arg, int totalRegisters)
{
	int index = RegisterToIndex (arg.m_label);
	int remapIndex = FindRegister (index, totalRegisters);
	arg.m_label = IndexToRegister(remapIndex);
}


void dDataFlowGraph::dRegisterInterferenceGraph::SortRegisters(int totalRegisters)
{
	qsort (m_registenFrequency, totalRegisters, 2 * sizeof (int), Compare);
	if (m_flowGraph->m_returnType != dCIL::m_void) {
		dAssert (0);
		int index = FindRegister (D_RETURN_REGISTER_INDEX, totalRegisters);
	}
}

int dDataFlowGraph::dRegisterInterferenceGraph::ColorGraph (int registerCount)
{
	dList<dTreeNode*> registerOrder;
	for (int i = 0; i < GetCount(); i ++) {
		dTreeNode* const bestNode = GetBestNode();
		registerOrder.Addtop(bestNode);
		dRegisterInterferenceNode& node = bestNode->GetInfo();
		dList<dRegisterInterferenceNodeEdge>& edgeListInfo = node.m_interferanceEdge; 
		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfo.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			if (edge.m_mark == 0) {
				edge.m_mark = 1;
				edge.m_twin->m_mark = 1;
			}
		}
	} 

	int registersUsed = -1;
	for (dList<dTreeNode*>::dListNode* node = registerOrder.GetFirst(); node; node = node->GetNext()) {
		dTreeNode* const varNode = node->GetInfo();
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		if (variable.m_registerIndex == -1) {
			int regMask = 0;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = variable.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
				dRegisterInterferenceNode& otherVariable = edge.m_targetNode->GetInfo();
				if (otherVariable.m_registerIndex != -1) {
					regMask |= (1 << otherVariable.m_registerIndex);
				}
			}

			int index = 0;
			for (int mask = regMask; mask & 1 ; mask >>= 1) {
				index ++;
			}
			variable.m_registerIndex = index;
		}
		m_flowGraph->m_registersUsedMask |= (1 << variable.m_registerIndex);
		registersUsed = dMax (registersUsed, variable.m_registerIndex);
	}
	return registersUsed + 1;
}

dDataFlowGraph::dRegisterInterferenceGraph::dTreeNode* dDataFlowGraph::dRegisterInterferenceGraph::GetBestNode()
{
#if 0
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dRegisterInterferenceNode& info = node->GetInfo();

		if (!info.m_inSet) {
			int count = 0;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = info.m_interferanceEdge.GetFirst(); edgeNode && (count < m_registerCount); edgeNode = edgeNode->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				if (!edge.m_mark) {
					count += (edge.m_isMov) ? 0 : 1;
				}
			}

			if (count < m_registerCount) {
				info.m_inSet = true;
				return node;
			}
		}
	}

	// we may have a potential spill, try to select a good spill node candidate
	int maxCount = 0;
	dTreeNode* bestSpillNode = NULL;
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dRegisterInterferenceNode& info = node->GetInfo();
		if (!info.m_inSet) {
			int count = 0;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = info.m_interferanceEdge.GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				if (!edge.m_mark) {
					count ++;
				}
			}
			if (count > maxCount) {
				maxCount = count;
				bestSpillNode = node;
			}
		}
	}
	dRegisterInterferenceNode& info = bestSpillNode->GetInfo();
	info.m_inSet = true;
	return bestSpillNode;

#else
	int minCount = 0x7fffff;
	dTreeNode* bestNode = NULL;
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dRegisterInterferenceNode& info = node->GetInfo();
		if (!info.m_inSet) {
			int count = 0;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = info.m_interferanceEdge.GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				if (!edge.m_mark) {
					count ++;
				}
			}
			if (count < minCount) {
				minCount = count;
				bestNode = node;
			}
		}
	}

	dRegisterInterferenceNode& info = bestNode->GetInfo();
	info.m_inSet = true;
	return bestNode;
#endif
}

dString dDataFlowGraph::GetRegisterName (dRegisterInterferenceGraph& interferenceGraph, const dString& varName) const
{
	if (varName[0] == '_') {
		return varName;
	} else {
		dRegisterInterferenceGraph::dTreeNode* const node = interferenceGraph.Find (varName);
		dRegisterInterferenceNode& var = node->GetInfo();
		return IndexToRegister(var.m_registerIndex);
	}
}



void dDataFlowGraph::AllocateRegisters (dRegisterInterferenceGraph& interferenceGraph)
{
	for (dCIL::dListNode* node = m_function; node; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();	

		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_assigment:
			{
				stmt.m_arg0.m_label = GetRegisterName (interferenceGraph, stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (interferenceGraph, stmt.m_arg1.m_label);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						stmt.m_arg2.m_label = GetRegisterName (interferenceGraph, stmt.m_arg2.m_label);
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_free:
			case dTreeAdressStmt::m_paramLoad:
			{
				stmt.m_arg0.m_label = GetRegisterName (interferenceGraph, stmt.m_arg0.m_label);
				break;
			}


			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_store:
			{
				stmt.m_arg0.m_label = GetRegisterName (interferenceGraph, stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (interferenceGraph, stmt.m_arg1.m_label);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg2.m_label = GetRegisterName (interferenceGraph, stmt.m_arg2.m_label);
				} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}
				break;
			}


			case dTreeAdressStmt::m_alloc:
			{
				stmt.m_arg0.m_label = GetRegisterName (interferenceGraph, stmt.m_arg0.m_label);
				stmt.m_arg1.m_label = GetRegisterName (interferenceGraph, stmt.m_arg1.m_label);
				break;
			}


			case dTreeAdressStmt::m_if:
			{
				stmt.m_arg0.m_label = GetRegisterName (interferenceGraph, stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (interferenceGraph, stmt.m_arg1.m_label);
				}
				break;
			}

			case dTreeAdressStmt::m_enter:
			{
				int returnRegisterMask = ~((m_returnType == 1) ? (1 << D_RETURN_REGISTER_INDEX) : 0);
				int regMask = m_registersUsedMask & returnRegisterMask; 
				stmt.m_extraInformation = regMask;
				break;
			}

			case dTreeAdressStmt::m_leave:
			{
				int returnRegisterMask = ~((m_returnType == 1) ? (1 << D_RETURN_REGISTER_INDEX) : 0);
				int regMask = m_registersUsedMask & returnRegisterMask; 
				stmt.m_extraInformation = regMask;
				break;
			}

			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_argument:
			case dTreeAdressStmt::m_function:
				break;
	
			default:
				dAssert (0);
		}
	}
}




void dDataFlowGraph::GetLoops (dList<dLoop>& loops) const
{

	for (dCIL::dListNode* node = m_function; node; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();	
		if ((stmt.m_instruction == dTreeAdressStmt::m_nop) && (stmt.m_arg2.m_label == D_LOOP_HEADER_SYMBOL)) {
			dList<dLoop>::dListNode* const loopNode = loops.Addtop();
			dLoop& loop = loopNode->GetInfo();
			loop.m_head = node;

			int order = stmt.m_extraInformation;
			for (dList<dLoop>::dListNode* root = loops.GetLast(); root != loops.GetFirst(); root = root->GetPrev()) {
				int code = root->GetInfo().m_head->GetInfo().m_extraInformation;
				if (code > order) {
					loops.InsertAfter (root, loopNode);
					break;
				}
			}

			for (dCIL::dListNode* node1 = node->GetNext(); node1; node1 = node1->GetNext()) {
				dTreeAdressStmt& stmt = node1->GetInfo();
				int code = stmt.m_extraInformation;
				if ((stmt.m_instruction == dTreeAdressStmt::m_nop) && (code == order) && (stmt.m_arg2.m_label == D_LOOP_TAIL_SYMBOL)) {
					loop.m_tail = node1;
					break;
				}
			}
		}
	}
}


bool dDataFlowGraph::IsStatementInReachList(dCIL::dListNode* const node, dList<dCIL::dListNode*>& definitionList, dCIL::dListNode* const me) const
{
	const dDataFlowPoint& point = m_dataFlowGraph.Find (node)->GetInfo();
	for (dList<dCIL::dListNode*>::dListNode* ptr = definitionList.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dCIL::dListNode* const defVarStmt = ptr->GetInfo();
		if (defVarStmt != me) {
			if (point.m_reachStmtInputSet.Find(defVarStmt)) {
				return true;
			}
		}
	}
	return false;
}

bool dDataFlowGraph::CheckBackEdgePreReachInLoop(dCIL::dListNode* const stmtNode, const dLoop& loop) const
{
	bool ret = false;
	const dString& var = stmtNode->GetInfo().m_arg0.m_label;
	dList<dCIL::dListNode*>& statementList = m_variableDefinitions.Find (var)->GetInfo();

	for (dCIL::dListNode* node = loop.m_head; !ret && (node != loop.m_tail->GetPrev()); node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_if:
			case dTreeAdressStmt::m_assigment:
			{
				if ((stmt.m_arg1.m_label == var) || (stmt.m_arg2.m_label == var)) {
					ret = IsStatementInReachList(node, statementList, stmtNode);
				}
				break;
			}

			case dTreeAdressStmt::m_store:
			{
				if ((stmt.m_arg0.m_label == var) || (stmt.m_arg1.m_label == var) || (stmt.m_arg2.m_label == var)) {
					ret = IsStatementInReachList(node, statementList, stmtNode);
				}
				break;
			}
		

			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_label:
				break;

			default:
				dAssert (0);
		}
	}
	return ret;
}


bool dDataFlowGraph::ApplyLoopOptimization(dLoop& loop)
{
	bool ret = false;

	dAssert (0);
/*
	// calculate the dominator tree for this loop
	dTree <dDominator, dCIL::dListNode*> dominatorTree;
	dDominator& rootDominator = dominatorTree.Insert (loop.m_head)->GetInfo(); 
	rootDominator.Insert (0, loop.m_head);
	for (dCIL::dListNode* node = loop.m_head->GetNext(); node != loop.m_tail; node = node->GetNext()) {
		dDominator& dominator = dominatorTree.Insert (node)->GetInfo(); 
		for (dCIL::dListNode* node1 = loop.m_head; node1 != loop.m_tail; node1 = node1->GetNext()) {
			dominator.Insert (0, node1);
		}
	}

	bool someSetChanged = false;
	while (!someSetChanged) {
		someSetChanged = true;

		for (dCIL::dListNode* node = loop.m_head->GetNext(); node != loop.m_tail; node = node->GetNext()) {
			dAssert (m_dataFlowGraph.Find(node));
			dDominator& dominator = dominatorTree.Find (node)->GetInfo(); 
			dDominator oldDominator (dominator);

			dDataFlowPoint& info = m_dataFlowGraph.Find(node)->GetInfo();
			for (dList<dDataFlowPoint*>::dListNode* predecessorsNode = info.m_predecessors.GetFirst(); predecessorsNode; predecessorsNode = predecessorsNode->GetNext()) {

				dDataFlowPoint* const predecessorsInfo = predecessorsNode->GetInfo();
				dCIL::dListNode* const stmtNode = predecessorsInfo->m_statement;
				dDominator& parentDominator = dominatorTree.Find (stmtNode)->GetInfo(); 
				dominator.Intersection (parentDominator);
			}
			dominator.Insert (node);

			someSetChanged = (someSetChanged && dominator.Compare(oldDominator));
		}
	}


#ifdef _DEBUG
	for (dCIL::dListNode* node = loop.m_head; node != loop.m_tail; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();	
		dTrace (("%d ", stmt.m_debug));
		DTRACE_INTRUCTION(&stmt);
	}

	dTrace (("\n\n"));

	for (dCIL::dListNode* node = loop.m_head; node != loop.m_tail; node = node->GetNext()) {
		dDominator& dominator = dominatorTree.Find (node)->GetInfo(); 
		dTreeAdressStmt& stmt = node->GetInfo();	
		dTrace (("%d: ", stmt.m_debug));

		dDominator::Iterator iter (dominator);
		for (iter.Begin(); iter; iter ++) {
			dCIL::dListNode* const dominatedNode = iter.GetKey();
			dTreeAdressStmt& stmt = dominatedNode->GetInfo();	
			dTrace (("%d ", stmt.m_debug));
		}
		dTrace (("\n"));
	}
#endif

	
	// mark all potential loop invariant candidates for code promotion	
	dTree <dDominator, dCIL::dListNode*>::Iterator iter (dominatorTree);
	for (iter.Begin(); iter; iter ++) {
		dDominator& dominator = iter.GetNode()->GetInfo();
		dTreeAdressStmt& stmt = iter.GetKey()->GetInfo();

		switch (stmt.m_instruction) 
		{
			case dTreeAdressStmt::m_assigment:
			{
				if (stmt.m_operator == dTreeAdressStmt::m_nothing) {	
					if ((stmt.m_arg1.m_type == dTreeAdressStmt::m_intConst) || (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatConst)) {
						dominator.m_isLoopInvariant = true;
					} else {
						dAssert (0);
					}
				} else {
					//dAssert (0);
				}
				break;
			}

			case dTreeAdressStmt::m_paramLoad:
			{
				dominator.m_isLoopInvariant = true;
				break;
			}
		}
	}

	CalculateLiveInputLiveOutput();
	UpdateReachingDefinitions();

	dDataFlowPoint& liveInfoAtTail = m_dataFlowGraph.Find(loop.m_tail)->GetInfo();
	dDataFlowPoint::dVariableSet<dString>& liveOut = liveInfoAtTail.m_liveOutputSet;


	dDominator& loopExitDominators = dominatorTree.Find (loop.m_tail->GetPrev()->GetPrev())->GetInfo(); 
	dCIL::dListNode* preHeaderNode = loop.m_head->GetPrev();
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = loop.m_head; node != loop.m_tail->GetPrev(); node = nextNode) {
		nextNode = node->GetNext();

		dDominator& dominator = dominatorTree.Find (node)->GetInfo(); 
		if (dominator.m_isLoopInvariant) {
			dTreeAdressStmt& stmt = node->GetInfo();
			switch (stmt.m_instruction) 
			{
				case dTreeAdressStmt::m_assigment:
				{
					if (stmt.m_operator == dTreeAdressStmt::m_nothing) {
						if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intConst) {
							if (loopExitDominators.Find (node) || !liveOut.Find (stmt.m_arg0.m_label)) {
								dAssert (m_variableDefinitions.Find (stmt.m_arg0.m_label));
								dList<dCIL::dListNode*>& varDefintionList = m_variableDefinitions.Find (stmt.m_arg0.m_label)->GetInfo();
								if ((varDefintionList.GetCount() == 1) || !CheckBackEdgePreReachInLoop(node, loop)) {
									ret = 1;
									dCIL::dListNode* const movedNode = m_cil->AppendAfter (preHeaderNode);
									movedNode->GetInfo() = stmt;
									stmt.m_instruction = dTreeAdressStmt::m_nop;
									stmt.m_arg0.m_label = "";
									stmt.m_arg1.m_label = "";
									stmt.m_arg2.m_label = "";
									stmt.m_extraInformation = 0;

									preHeaderNode = movedNode;
									loop.m_head = loop.m_head->GetNext();
								}
							}
						}
					}
					break;
				}

				case dTreeAdressStmt::m_paramLoad:
				{
					if (loopExitDominators.Find (node) || !liveOut.Find (stmt.m_arg0.m_label)) {
						dAssert (m_variableDefinitions.Find (stmt.m_arg0.m_label));
						dList<dCIL::dListNode*>& varDefintionList = m_variableDefinitions.Find (stmt.m_arg0.m_label)->GetInfo();
						if ((varDefintionList.GetCount() == 1) || !CheckBackEdgePreReachInLoop(node, loop)) {
							if (!CheckBackEdgePreReachInLoop(node, loop)) {
								ret = 1;
								dCIL::dListNode* const movedNode = m_cil->AppendAfter (preHeaderNode);
								movedNode->GetInfo() = stmt;
								stmt.m_instruction = dTreeAdressStmt::m_nop;
								stmt.m_arg0.m_label = "";
								stmt.m_arg1.m_label = "";
								stmt.m_arg2.m_label = "";
								stmt.m_extraInformation = 0;

								preHeaderNode = movedNode;
								loop.m_head = loop.m_head->GetNext();
							}
						}
					}
					break;
				}
			}
		}
	}
*/
	return ret;
}

void dDataFlowGraph::ApplyLocalOptimizations()
{

m_cil->Trace();

	m_mark += 2;
	BuildBasicBlockGraph();
m_basicBlocks.Trace();

	ApplyIfStatementsSimplification();
m_basicBlocks.Trace();

	CalculateReachingDefinitions();
	for (bool optimized = true; optimized;) {
		optimized = false;
		optimized |= ApplySubExpresionToCopyPropagation();
//m_cil->Trace();
m_basicBlocks.Trace();
		if (optimized) {
			ApplyRemoveDeadCode();
//m_cil->Trace();
m_basicBlocks.Trace();
		}
	}

	// apply generic data flow optimization for single blocks
	for (bool optimized = true; optimized;) {
		optimized = false;
		UpdateReachingDefinitions();

		optimized |= ApplyConstantPropagation();
//m_cil->Trace();
m_basicBlocks.Trace();
		optimized |= ApplyCopyPropagation();
//m_cil->Trace();
m_basicBlocks.Trace();
		optimized |= ApplyConstantFolding();
//m_cil->Trace();
m_basicBlocks.Trace();
		optimized |= ApplyInstructionSematicOrdering();
//m_cil->Trace();
m_basicBlocks.Trace();
		optimized |= ApplyRemoveDeadCode();
//m_cil->Trace();
m_basicBlocks.Trace();
	}

	// apply peephole optimizations
	for (bool optimized = true; optimized;) {
		optimized = false;
		UpdateReachingDefinitions();

		optimized |= ApplySubExpresionToCopyPropagation();
//m_cil->Trace();
m_basicBlocks.Trace();
		if (optimized) {
			ApplyRemoveDeadCode();
//m_cil->Trace();
m_basicBlocks.Trace();
		}
	}

	for (bool optimized = true; optimized;) {
		optimized = false;
		UpdateReachingDefinitions();

		optimized |= ApplyCommonSubExpresion();
//m_cil->Trace();
m_basicBlocks.Trace();

		optimized |= ApplyCopyPropagation();
//m_cil->Trace();
m_basicBlocks.Trace();
		optimized |= ApplyRemoveDeadCode();
//m_cil->Trace();
m_basicBlocks.Trace();
	}

/*
	// apply loops optimizations here
	dList<dLoop> loops;
	GetLoops (loops);
	for (bool optimized = true; optimized;) {
		optimized = false;
		for (dList<dLoop>::dListNode* loopNode = loops.GetFirst(); loopNode; loopNode = loopNode->GetNext()) {
			dLoop& loop = loopNode->GetInfo();
			optimized |= ApplyLoopOptimization(loop);
m_cil->Trace();
		}
	}
*/

}


void dDataFlowGraph::RegistersAllocation (int registerCount)
{
	dRegisterInterferenceGraph interferenceGraph(this, registerCount);
}

