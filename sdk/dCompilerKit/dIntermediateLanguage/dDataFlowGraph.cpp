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
#include "dCILInstrBranch.h"
#include "dCILInstrLoadStore.h"
#include "dCILInstrArithmetic.h"
#include "dRegisterInterferenceGraph.h"






dDataFlowGraph::dDataFlowGraph (dCIL* const cil, dCIL::dListNode* const function)
	:m_cil (cil)
	,m_function(function)
	,m_mark (0)
{
//	BuildBasicBlockGraph();

//m_basicBlocks.Trace();
}

dDataFlowGraph::~dDataFlowGraph(void)
{
}

/*
void dDataFlowGraph::CalculateSuccessorsAndPredecessors()
{
dAssert (0);

	m_dataFlowGraph.RemoveAll();
	dTree<dBasicBlock*, dCIL::dListNode*> blockMap;
	for (dBasicBlocksList::dListNode* blockNode = m_basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();

		//block.Trace();
		blockMap.Insert(&block, block.m_begin);
		//		
		bool terminate = false;
		for (dCIL::dListNode* node = block.m_begin; !terminate; node = node->GetNext()) {
			terminate = (node == block.m_end);
			dAssert(!m_dataFlowGraph.Find(node));
			dDataFlowPoint& dataPoint = m_dataFlowGraph.Insert(node)->GetInfo();
			dataPoint.Init(node, blockNode);
		};

		for (dCIL::dListNode* node = block.m_begin; node != block.m_end; node = node->GetNext()) {
			dDataFlowPoint& graphStatement = m_dataFlowGraph.Find(node)->GetInfo();
			graphStatement.m_successors.Append(&m_dataFlowGraph.Find(node->GetNext())->GetInfo());
		}

		for (dCIL::dListNode* node = block.m_end; node != block.m_begin; node = node->GetPrev()) {
			dDataFlowPoint& graphStatement = m_dataFlowGraph.Find(node)->GetInfo();
			graphStatement.m_predecessors.Append(&m_dataFlowGraph.Find(node->GetPrev())->GetInfo());
		}
	}

	//m_cil->Trace();
	m_mark += 1;
	dList<dBasicBlock*> stack;
	stack.Append(&m_basicBlocks.GetFirst()->GetInfo());
	while (stack.GetCount()) {
		dBasicBlock* const block = stack.GetLast()->GetInfo();

		stack.Remove(stack.GetLast()->GetInfo());
		if (block->m_mark < m_mark) {
			block->m_mark = m_mark;
			//m_traversalBlocksOrder.Addtop(block);
			//block->Trace();

			dCILInstr* const instruction = block->m_end->GetInfo();
			dAssert(instruction->IsBasicBlockEnd());
			if (instruction->GetAsIF()) {
				dCILInstrConditional* const ifInstr = instruction->GetAsIF();
				dAssert(m_dataFlowGraph.Find(block->m_end->GetPrev()));
				dAssert(m_dataFlowGraph.Find(ifInstr->GetTrueTarget()));
				dAssert(m_dataFlowGraph.Find(ifInstr->GetFalseTarget()));
				dAssert(blockMap.Find(ifInstr->GetTrueTarget()));
				dAssert(blockMap.Find(ifInstr->GetFalseTarget()));
				dAssert(m_dataFlowGraph.Find(block->m_end->GetNext()));
				dAssert(blockMap.Find(block->m_end->GetNext()));

				dDataFlowPoint* const graphStatement = &m_dataFlowGraph.Find(block->m_end)->GetInfo();
				dDataFlowPoint* const child0 = &m_dataFlowGraph.Find(ifInstr->GetTrueTarget())->GetInfo();
				dDataFlowPoint* const child1 = &m_dataFlowGraph.Find(ifInstr->GetFalseTarget())->GetInfo();

				graphStatement->m_successors.Append(child0);
				graphStatement->m_successors.Append(child1);
				child0->m_predecessors.Append(graphStatement);
				child1->m_predecessors.Append(graphStatement);

				stack.Append(blockMap.Find(ifInstr->GetTrueTarget())->GetInfo());
				stack.Append(blockMap.Find(ifInstr->GetFalseTarget())->GetInfo());

			}
			else if (instruction->GetAsGoto()) {
				dCILInstrGoto* const gotoInst = instruction->GetAsGoto();

				dAssert(m_dataFlowGraph.Find(block->m_end));
				dAssert(m_dataFlowGraph.Find(gotoInst->GetTarget()));
				dAssert(blockMap.Find(gotoInst->GetTarget()));

				dDataFlowPoint* const graphStatement = &m_dataFlowGraph.Find(block->m_end)->GetInfo();
				dDataFlowPoint* const child0 = &m_dataFlowGraph.Find(gotoInst->GetTarget())->GetInfo();

				graphStatement->m_successors.Append(child0);
				child0->m_predecessors.Append(graphStatement);

				stack.Append(blockMap.Find(gotoInst->GetTarget())->GetInfo());
			}
		}
	}

}

void dDataFlowGraph::DeleteUnreachedBlocks()
{
	dTree<dBasicBlock*, dCIL::dListNode*> blockMap;
	for (dBasicBlocksList::dListNode* blockNode = m_basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		blockMap.Insert(&block, block.m_begin);
	}

	m_mark += 1;
	dList<dBasicBlock*> stack;
	stack.Append(&m_basicBlocks.GetFirst()->GetInfo());
	while (stack.GetCount()) {
		dBasicBlock* const block = stack.GetLast()->GetInfo();

		stack.Remove(stack.GetLast()->GetInfo());
		if (block->m_mark < m_mark) {
			block->m_mark = m_mark;

			dCILInstr* const instruction = block->m_end->GetInfo();
			dAssert(instruction->IsBasicBlockEnd());
			if (instruction->GetAsIF()) {
				dCILInstrConditional* const ifInstr = instruction->GetAsIF();
				stack.Append(blockMap.Find(ifInstr->GetTrueTarget())->GetInfo());
				stack.Append(blockMap.Find(ifInstr->GetFalseTarget())->GetInfo());

			} else if (instruction->GetAsGoto()) {
				dCILInstrGoto* const gotoInst = instruction->GetAsGoto();
				stack.Append(blockMap.Find(gotoInst->GetTarget())->GetInfo());
			}
		}
	}

	dBasicBlocksList::dListNode* nextBlockNode;
	for (dBasicBlocksList::dListNode* blockNode = m_basicBlocks.GetFirst(); blockNode; blockNode = nextBlockNode) {
		dBasicBlock& block = blockNode->GetInfo();
		nextBlockNode = blockNode->GetNext();
		if (block.m_mark != m_mark) {
			//block.Trace();
			bool terminate = false;
			dCIL::dListNode* nextNode;
			for (dCIL::dListNode* node = block.m_begin; !terminate; node = nextNode) {
				terminate = (node == block.m_end);
				nextNode = node->GetNext();
				m_cil->Remove(node);
			}
			m_basicBlocks.Remove(blockNode);
		}
	}
}
*/

/*
void dDataFlowGraph::BuildBasicBlockGraph()
{
	m_basicBlocks.Build (m_function);
}


void dDataFlowGraph::ConvertToSSA ()
{
//m_cil->Trace();
	m_basicBlocks.ConvertToSSA (this);
//m_cil->Trace();
//m_basicBlocks.Trace();
}
*/

#if 0

void dDataFlowGraph::dBasicBlock::Trace() const
{
	#ifdef TRACE_INTERMEDIATE_CODE
		int test = 1;
		for (dCIL::dListNode* stmtNode = m_begin; test; stmtNode = stmtNode->GetNext()) {
			test = (stmtNode != m_end);
			const dThreeAdressStmt& stmt = stmtNode->GetInfo();
			DTRACE_INTRUCTION(&stmt);
		}
		dTrace(("\n"));
	#endif
}

int dDataFlowGraph::EvaluateBinaryExpression (const dString& arg1, dThreeAdressStmt::dOperator operation, const dString& arg2) const
{
	int operando = 0;
	int operando1 = arg1.ToInteger();
	int operando2 = arg2.ToInteger();

	switch (operation) 
	{
		case dThreeAdressStmt::m_add:
			operando = operando1 + operando2;
			break;

		case dThreeAdressStmt::m_greather:
			operando = operando1 > operando2;
			break;

		case dThreeAdressStmt::m_less:
			operando = operando1 < operando2;
			break;

		case dThreeAdressStmt::m_sub:
			operando = operando1 - operando2;
			break;

		case dThreeAdressStmt::m_mul:
			operando = operando1 * operando2;
			break;

		case dThreeAdressStmt::m_div:
			operando = operando1 / operando2;
			break;

		case dThreeAdressStmt::m_mod:
			operando = operando1 % operando2;
			break;

		case dThreeAdressStmt::m_equal:
			operando = (operando1 = operando2);
			break;

		case dThreeAdressStmt::m_identical:
			operando = (operando1 == operando2);
			break;

		case dThreeAdressStmt::m_different:
			operando = (operando1 != operando2);
			break;

		case dThreeAdressStmt::m_lessEqual:
			operando = (operando1 <= operando2);
			break;

		case dThreeAdressStmt::m_greatherEqual:
			operando = (operando1 >= operando2);
			break;

		case dThreeAdressStmt::m_operatorsCount:

		default:
			dAssert (0);
	}
	return operando;
}




bool dDataFlowGraph::ApplyConstantPropagation()
{
	bool ret = false;
	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dThreeAdressStmt& stmt = stmtNode->GetInfo();
		if ((stmt.m_instruction == dThreeAdressStmt::m_assigment) && (stmt.m_operator == dThreeAdressStmt::m_nothing) && (stmt.m_arg1.m_type == dThreeAdressStmt::m_intConst)) {
			for (dCIL::dListNode* stmtNode1 = stmtNode->GetNext(); stmtNode1; stmtNode1 = stmtNode1->GetNext()) {
				dThreeAdressStmt& stmt1 = stmtNode1->GetInfo();
				switch (stmt1.m_instruction)
				{
					case dThreeAdressStmt::m_assigment:
					{
						if (stmt1.m_operator == dThreeAdressStmt::m_nothing) {
							if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoMoveReachInstruction(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg1 = stmt.m_arg1;
								UpdateReachingDefinitions();
							}
						} else if (m_cil->m_commutativeOperator[stmt1.m_operator]) {
							
							if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoMoveReachInstruction(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg1 = stmt1.m_arg2;
								stmt1.m_arg2 = stmt.m_arg1;
								if ((stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoMoveReachInstruction(stmtNode1, stmtNode)) {
									stmt1.m_arg1 = stmt.m_arg1;
								}
								UpdateReachingDefinitions();
							} else if ((stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoMoveReachInstruction(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg2 = stmt.m_arg1;
								UpdateReachingDefinitions();
							}
							
						} else {
							if ((stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoMoveReachInstruction(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg2 = stmt.m_arg1;
								UpdateReachingDefinitions();
							}

							if ((stmt1.m_arg2.m_type == dThreeAdressStmt::m_intConst) && (stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoMoveReachInstruction(stmtNode1, stmtNode)) {
								ret = true;
								stmt1.m_arg1 = stmt.m_arg1;
								UpdateReachingDefinitions();
							}
						}

/*
						if ((stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)){
							if ((stmt1.m_operator != dThreeAdressStmt::m_nothing) && m_cil->m_commutativeOperator[stmt1.m_operator]) {
							} else {
							ret = true;
							stmt1.m_arg1 = stmt.m_arg1;
						}
						if ((stmt1.m_operator != dThreeAdressStmt::m_nothing) && (stmt1.m_arg2.m_label == stmt.m_arg0.m_label) && DoStatementAreachesStatementB(stmtNode1, stmtNode)) {
							ret = true;
							stmt1.m_arg2 = stmt.m_arg1;
						}
*/
						break;
					}

					case dThreeAdressStmt::m_if:
					{
						if ((stmt1.m_arg0.m_label == stmt.m_arg0.m_label) && (stmt1.m_arg1.m_type == dThreeAdressStmt::m_intConst) && DoMoveReachInstruction(stmtNode1, stmtNode)) {
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
		dThreeAdressStmt& stmt = stmtNode->GetInfo();
		if ((stmt.m_instruction == dThreeAdressStmt::m_if) && (stmt.m_operator == dThreeAdressStmt::m_identical) || (stmt.m_operator == dThreeAdressStmt::m_different)) {
			dCIL::dListNode* const prevStmtNode = stmtNode->GetPrev();
			dThreeAdressStmt& stmtA = prevStmtNode->GetInfo();
			if ((stmt.m_arg1.m_label == stmtA.m_arg0.m_label) && (stmtA.m_instruction == dThreeAdressStmt::m_assigment)  &&  (stmtA.m_operator == dThreeAdressStmt::m_nothing) && (stmtA.m_arg1.m_type == dThreeAdressStmt::m_intConst)) {
				dCIL::dListNode* const prevPrevStmtNode = prevStmtNode->GetPrev();
				dThreeAdressStmt& stmtB = prevPrevStmtNode->GetInfo();
				if ((stmt.m_arg0.m_label == stmtB.m_arg0.m_label) && (stmtB.m_instruction == dThreeAdressStmt::m_assigment)  && (m_cil->m_conditionals[stmtB.m_operator]) && (stmtB.m_arg2.m_type != dThreeAdressStmt::m_intConst)) {
					stmt.m_arg0 = stmtB.m_arg1;
					stmt.m_arg1 = stmtB.m_arg2;
					if (stmt.m_operator == dThreeAdressStmt::m_identical) {
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




bool dDataFlowGraph::ApplySubExpresionToCopyPropagation()
{
	bool ret = false;
	for (dCIL::dListNode* stmtNode = m_function; stmtNode; stmtNode = stmtNode->GetNext()) {
		dThreeAdressStmt& stmt = stmtNode->GetInfo();
		if (stmt.m_instruction == dThreeAdressStmt::m_assigment) {
			if ((stmt.m_operator != dThreeAdressStmt::m_nothing) && (stmt.m_arg0.m_label != stmt.m_arg1.m_label) && (stmt.m_arg0.m_label != stmt.m_arg2.m_label)){
				for (dCIL::dListNode* stmtNode1 = stmtNode->GetNext(); stmtNode1; stmtNode1 = stmtNode1->GetNext()) {
					dThreeAdressStmt& stmt1 = stmtNode1->GetInfo();
					switch (stmt1.m_instruction) 
					{
						case dThreeAdressStmt::m_assigment:
						{
							if ((stmt1.m_operator == dThreeAdressStmt::m_nothing) && (stmt1.m_arg1.m_label == stmt.m_arg0.m_label) && DoMoveReachInstruction(stmtNode1, stmtNode) ){
								ret = true;
								stmt1.m_operator = stmt.m_operator;
								stmt1.m_arg1 = stmt.m_arg1;
								stmt1.m_arg2 = stmt.m_arg2;
								UpdateReachingDefinitions();
							}
							break;
						}

						case dThreeAdressStmt::m_if:
						{
							if (m_cil->m_conditionals[stmt.m_operator] && (stmt1.m_arg1.m_label == "0") && (stmt1.m_arg0.m_label == stmt.m_arg0.m_label) && DoMoveReachInstruction(stmtNode1, stmtNode)) {
								switch (stmt1.m_operator) 
								{
									case dThreeAdressStmt::m_identical:
									{
										ret = true;
										stmt1.m_operator = m_cil->m_operatorComplement[stmt.m_operator];
										stmt1.m_arg0 = stmt.m_arg1;
										stmt1.m_arg1 = stmt.m_arg2;
										UpdateReachingDefinitions();
										break;
									}

									case dThreeAdressStmt::m_different:
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
		dThreeAdressStmt& stmt = stmtNode->GetInfo();
		if ((stmt.m_instruction == dThreeAdressStmt::m_assigment) && (stmt.m_operator != dThreeAdressStmt::m_nothing)) {
			for (dCIL::dListNode* stmtNode1 = stmtNode->GetNext(); stmtNode1; stmtNode1 = stmtNode1->GetNext()) {
				dThreeAdressStmt& stmt1 = stmtNode1->GetInfo();
				if ((stmt1.m_instruction == dThreeAdressStmt::m_assigment) && (stmt1.m_operator == stmt.m_operator)) {
					if ((stmt.m_arg1.m_label == stmt1.m_arg1.m_label) && (stmt.m_arg2.m_label == stmt1.m_arg2.m_label)) {
						if (DoMoveReachInstruction(stmtNode1, stmtNode)) {
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
		dThreeAdressStmt& stmt = stmtNode->GetInfo();

		switch  (stmt.m_instruction) 
		{
			case dThreeAdressStmt::m_if:
			{
				if (stmt.m_arg1.m_type == dThreeAdressStmt::m_intConst) {
					ret = true;
					int val = EvaluateBinaryExpression (stmt.m_arg0.m_label, stmt.m_operator, stmt.m_arg1.m_label);
					if (val) {
						stmt.m_instruction = dThreeAdressStmt::m_goto;
						stmt.m_arg0.m_label = stmt.m_arg2.m_label;
					} else {
						stmt.m_instruction = dThreeAdressStmt::m_nop;
					}
				} else if ((stmt.m_arg0.m_type == dThreeAdressStmt::m_intVar) && (stmt.m_arg1.m_type == dThreeAdressStmt::m_intVar)) {
					dList<dCIL::dListNode*>& argListNodeA = m_variableDefinitions.Find (stmt.m_arg0.m_label)->GetInfo();
					dList<dCIL::dListNode*>& argListNodeB = m_variableDefinitions.Find (stmt.m_arg1.m_label)->GetInfo();
					if ((argListNodeA.GetCount() == 1) && (argListNodeB.GetCount() == 1)) {
						dThreeAdressStmt& argStmtA = argListNodeA.GetFirst()->GetInfo()->GetInfo();
						dThreeAdressStmt& argStmtB = argListNodeB.GetFirst()->GetInfo()->GetInfo();
						if ((argStmtA.m_arg1.m_type == dThreeAdressStmt::m_intConst) && (argStmtB.m_arg1.m_type == dThreeAdressStmt::m_intConst)) {
							ret = true;
							stmt.m_arg0 = argStmtA.m_arg1;
							stmt.m_arg1 = argStmtB.m_arg1;
							int val = EvaluateBinaryExpression (stmt.m_arg0.m_label, stmt.m_operator, stmt.m_arg1.m_label);
							if (val) {
								stmt.m_instruction = dThreeAdressStmt::m_goto;
								stmt.m_arg0.m_label = stmt.m_arg2.m_label;
							} else {
								stmt.m_instruction = dThreeAdressStmt::m_nop;
							}
							UpdateReachingDefinitions();
//m_cil->Trace();
						}
					}
				}
				break;
			}

			case dThreeAdressStmt::m_assigment:
			{
				if (stmt.m_operator != dThreeAdressStmt::m_nothing) {
					if (((stmt.m_arg1.m_type == dThreeAdressStmt::m_intConst) || (stmt.m_arg1.m_type == dThreeAdressStmt::m_floatConst)) &&
						((stmt.m_arg2.m_type == dThreeAdressStmt::m_intConst) || (stmt.m_arg2.m_type == dThreeAdressStmt::m_floatConst))) {
						ret = true;
						if ((stmt.m_arg1.m_type == dThreeAdressStmt::m_intConst) && (stmt.m_arg1.m_type == dThreeAdressStmt::m_intConst)) {
							int arg0 = EvaluateBinaryExpression (stmt.m_arg1.m_label, stmt.m_operator, stmt.m_arg2.m_label);
							stmt.m_operator = dThreeAdressStmt::m_nothing;
							stmt.m_arg1.m_label = dString(arg0);
							stmt.m_arg2.m_label = dString ("");

						} else if ((stmt.m_arg1.m_type == dThreeAdressStmt::m_floatConst) && (stmt.m_arg1.m_type == dThreeAdressStmt::m_floatConst)) {
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





void dDataFlowGraph::GetLoops (dList<dLoop>& loops) const
{

	for (dCIL::dListNode* node = m_function; node; node = node->GetNext()) {
		dThreeAdressStmt& stmt = node->GetInfo();	
		if ((stmt.m_instruction == dThreeAdressStmt::m_nop) && (stmt.m_arg2.m_label == D_LOOP_HEADER_SYMBOL)) {
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
				dThreeAdressStmt& stmt = node1->GetInfo();
				int code = stmt.m_extraInformation;
				if ((stmt.m_instruction == dThreeAdressStmt::m_nop) && (code == order) && (stmt.m_arg2.m_label == D_LOOP_TAIL_SYMBOL)) {
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
		dThreeAdressStmt& stmt = node->GetInfo();
		switch (stmt.m_instruction)
		{
			case dThreeAdressStmt::m_if:
			case dThreeAdressStmt::m_assigment:
			{
				if ((stmt.m_arg1.m_label == var) || (stmt.m_arg2.m_label == var)) {
					ret = IsStatementInReachList(node, statementList, stmtNode);
				}
				break;
			}

			case dThreeAdressStmt::m_store:
			{
				if ((stmt.m_arg0.m_label == var) || (stmt.m_arg1.m_label == var) || (stmt.m_arg2.m_label == var)) {
					ret = IsStatementInReachList(node, statementList, stmtNode);
				}
				break;
			}
		

			case dThreeAdressStmt::m_nop:
			case dThreeAdressStmt::m_label:
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
		dThreeAdressStmt& stmt = node->GetInfo();	
		dTrace (("%d ", stmt.m_debug));
		DTRACE_INTRUCTION(&stmt);
	}

	dTrace (("\n\n"));

	for (dCIL::dListNode* node = loop.m_head; node != loop.m_tail; node = node->GetNext()) {
		dDominator& dominator = dominatorTree.Find (node)->GetInfo(); 
		dThreeAdressStmt& stmt = node->GetInfo();	
		dTrace (("%d: ", stmt.m_debug));

		dDominator::Iterator iter (dominator);
		for (iter.Begin(); iter; iter ++) {
			dCIL::dListNode* const dominatedNode = iter.GetKey();
			dThreeAdressStmt& stmt = dominatedNode->GetInfo();	
			dTrace (("%d ", stmt.m_debug));
		}
		dTrace (("\n"));
	}
#endif

	
	// mark all potential loop invariant candidates for code promotion	
	dTree <dDominator, dCIL::dListNode*>::Iterator iter (dominatorTree);
	for (iter.Begin(); iter; iter ++) {
		dDominator& dominator = iter.GetNode()->GetInfo();
		dThreeAdressStmt& stmt = iter.GetKey()->GetInfo();

		switch (stmt.m_instruction) 
		{
			case dThreeAdressStmt::m_assigment:
			{
				if (stmt.m_operator == dThreeAdressStmt::m_nothing) {	
					if ((stmt.m_arg1.m_type == dThreeAdressStmt::m_intConst) || (stmt.m_arg1.m_type == dThreeAdressStmt::m_floatConst)) {
						dominator.m_isLoopInvariant = true;
					} else {
						dAssert (0);
					}
				} else {
					//dAssert (0);
				}
				break;
			}

			case dThreeAdressStmt::m_paramLoad:
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
			dThreeAdressStmt& stmt = node->GetInfo();
			switch (stmt.m_instruction) 
			{
				case dThreeAdressStmt::m_assigment:
				{
					if (stmt.m_operator == dThreeAdressStmt::m_nothing) {
						if (stmt.m_arg1.m_type == dThreeAdressStmt::m_intConst) {
							if (loopExitDominators.Find (node) || !liveOut.Find (stmt.m_arg0.m_label)) {
								dAssert (m_variableDefinitions.Find (stmt.m_arg0.m_label));
								dList<dCIL::dListNode*>& varDefintionList = m_variableDefinitions.Find (stmt.m_arg0.m_label)->GetInfo();
								if ((varDefintionList.GetCount() == 1) || !CheckBackEdgePreReachInLoop(node, loop)) {
									ret = 1;
									dCIL::dListNode* const movedNode = m_cil->AppendAfter (preHeaderNode);
									movedNode->GetInfo() = stmt;
									stmt.m_instruction = dThreeAdressStmt::m_nop;
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

				case dThreeAdressStmt::m_paramLoad:
				{
					if (loopExitDominators.Find (node) || !liveOut.Find (stmt.m_arg0.m_label)) {
						dAssert (m_variableDefinitions.Find (stmt.m_arg0.m_label));
						dList<dCIL::dListNode*>& varDefintionList = m_variableDefinitions.Find (stmt.m_arg0.m_label)->GetInfo();
						if ((varDefintionList.GetCount() == 1) || !CheckBackEdgePreReachInLoop(node, loop)) {
							if (!CheckBackEdgePreReachInLoop(node, loop)) {
								ret = 1;
								dCIL::dListNode* const movedNode = m_cil->AppendAfter (preHeaderNode);
								movedNode->GetInfo() = stmt;
								stmt.m_instruction = dThreeAdressStmt::m_nop;
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
	m_mark += 2;
	BuildBasicBlockGraph();
//m_cil->Trace();

	ApplyIfStatementsSimplification();
//m_cil->Trace();

	CalculateReachingDefinitions();
	for (bool optimized = true; optimized;) {
		optimized = false;
		optimized |= ApplySubExpresionToCopyPropagation();
//m_cil->Trace();
//m_basicBlocks.Trace();
		if (optimized) {
			ApplyRemoveDeadCode();
//m_cil->Trace();
//m_basicBlocks.Trace();
		}
	}

	// apply generic data flow optimization for single blocks
	for (bool optimized = true; optimized;) {
		optimized = false;
		UpdateReachingDefinitions();

		optimized |= ApplyConstantPropagation();
//m_cil->Trace();
//m_basicBlocks.Trace();
		optimized |= ApplyCopyPropagation();
//m_cil->Trace();
//m_basicBlocks.Trace();
		optimized |= ApplyConstantFolding();
//m_cil->Trace();
//m_basicBlocks.Trace();
		optimized |= ApplySemanticInstructionReordering();
//m_cil->Trace();
//m_basicBlocks.Trace();
		optimized |= ApplyRemoveDeadCode();
//m_cil->Trace();
//m_basicBlocks.Trace();
	}

	// apply peephole optimizations
	for (bool optimized = true; optimized;) {
		optimized = false;
		UpdateReachingDefinitions();

		optimized |= ApplySubExpresionToCopyPropagation();
//m_cil->Trace();
//m_basicBlocks.Trace();
		if (optimized) {
			ApplyRemoveDeadCode();
//m_cil->Trace();
//m_basicBlocks.Trace();
		}
	}

	for (bool optimized = true; optimized;) {
		optimized = false;
		UpdateReachingDefinitions();

		optimized |= ApplyCommonSubExpresion();
//m_cil->Trace();
//m_basicBlocks.Trace();
		optimized |= ApplyCopyPropagation();
//m_cil->Trace();
//m_basicBlocks.Trace();
		optimized |= ApplyRemoveDeadCode();
//m_cil->Trace();
//m_basicBlocks.Trace();
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

void dDataFlowGraph::BuildDefinedAndKilledStatementSets()
{
	dTree<dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();
		point.m_killStmtSet.RemoveAll();
	}

	m_variableDefinitions.RemoveAll();
	for (dCIL::dListNode* ptr = m_basicBlocks.m_begin; ptr != m_basicBlocks.m_end; ptr = ptr->GetNext()) {
		dCILInstr* const instruc = ptr->GetInfo();
//instruc->Trace();
		instruc->AddDefinedVariable (m_variableDefinitions);

/*
		dThreeAdressStmt& stmt = ptr->GetInfo();	
//DTRACE_INTRUCTION (&stmt);		
		switch (stmt.m_instruction)
		{
			case dThreeAdressStmt::m_argument:
			case dThreeAdressStmt::m_assigment:
			{
				dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(stmt.m_arg0.m_label);
				node->GetInfo().Append(ptr);
				break;
			}

			case dThreeAdressStmt::m_loadBase:
			{
				dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(stmt.m_arg0.m_label);
				node->GetInfo().Append(ptr);
				break;
			}


#if 0
			case dThreeAdressStmt::m_load:
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
            
			case dThreeAdressStmt::m_store:
			{
				if (stmt.m_arg0.m_label == m_returnVariableName) {
					statementUsingReturnVariable.Insert(ptr);
				}
				break;
			}

            case dThreeAdressStmt::m_new:
            {
                dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(stmt.m_arg0.m_label);
                node->GetInfo().Append(ptr);

                if (stmt.m_arg1.m_label == m_returnVariableName) {
                    statementUsingReturnVariable.Insert(ptr);
                }
                break;
             }

            case dThreeAdressStmt::m_release:
            {
                if (stmt.m_arg0.m_label == m_returnVariableName) {
                    statementUsingReturnVariable.Insert(ptr);
                }
                break;
            }


			case dThreeAdressStmt::m_push:
			{
				if (stmt.m_arg1.m_label == m_returnVariableName) {
					statementUsingReturnVariable.Insert(ptr);
				}
				break;
			}

			case dThreeAdressStmt::m_if:
			{
				if (stmt.m_arg0.m_type == dThreeAdressStmt::m_intVar) {
					if (stmt.m_arg0.m_label == m_returnVariableName) {
						statementUsingReturnVariable.Insert(ptr);
					}
				}

				if (stmt.m_arg1.m_type == dThreeAdressStmt::m_intVar) {
					if (stmt.m_arg1.m_label == m_returnVariableName) {
						statementUsingReturnVariable.Insert(ptr);
					}
				}
				break;
			}

			
			{
				dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(stmt.m_arg2.m_label);
				node->GetInfo().Append(ptr);

//				if (stmt.m_arg0.m_label == m_returnVariableName) {
//					statementUsingReturnVariable.Insert(ptr);
//				}
//				break;
			}

#endif

			case dThreeAdressStmt::m_call:
			{
				if (stmt.m_arg0.GetType().m_isPointer || (stmt.m_arg0.GetType().m_intrinsicType != dThreeAdressStmt::m_void)) {
					dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const node = m_variableDefinitions.Insert(stmt.m_arg0.m_label);
					node->GetInfo().Append(ptr);
				}
				break;					
			}

			//case dThreeAdressStmt::m_pop:
			//case dThreeAdressStmt::m_enter:
			//case dThreeAdressStmt::m_leave:
			case dThreeAdressStmt::m_param:
			case dThreeAdressStmt::m_if:
			case dThreeAdressStmt::m_ifnot:
			case dThreeAdressStmt::m_ret:
			case dThreeAdressStmt::m_goto:
			case dThreeAdressStmt::m_nop:
			case dThreeAdressStmt::m_label:
			case dThreeAdressStmt::m_function:
			case dThreeAdressStmt::m_storeBase:
				break;

			default:
				dAssert (0);
		}
*/
	}

	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();

		point.m_generateStmt = false;
		//dThreeAdressStmt& stmt = point.m_statement->GetInfo();	
		dCILInstr* const instruc = point.m_statement->GetInfo();
//instruc->Trace();
		instruc->AddKilledStatements(m_variableDefinitions, point);
		
/*
//DTRACE_INTRUCTION (&stmt);		
		switch (stmt.m_instruction)
		{
			case dThreeAdressStmt::m_param:
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

//			case dThreeAdressStmt::m_storeBase:
//			case dThreeAdressStmt::m_loadBase:
//			{
//				point.m_generateStmt = true;
//				dAssert (m_variableDefinitions.Find(stmt.m_arg2.m_label));
//				dList<dCIL::dListNode*>& defsList = m_variableDefinitions.Find(stmt.m_arg2.m_label)->GetInfo();
//				for (dList<dCIL::dListNode*>::dListNode* defNode = defsList.GetFirst(); defNode; defNode = defNode->GetNext()) {
//					dCIL::dListNode* const killStement = defNode->GetInfo();
//					if (killStement != point.m_statement) {
//						point.m_killStmtSet.Insert(killStement);
//					}
//				}
//				break;
//			}
//			case dThreeAdressStmt::m_param:
//			{
//				point.m_generateStmt = true;
//				dAssert (m_variableDefinitions.Find(stmt.m_arg0.m_label));
//				dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const retRegisterNode = m_variableDefinitions.Find(stmt.m_arg0.m_label);
//				dList<dCIL::dListNode*>& defsList = retRegisterNode->GetInfo();
//				for (dList<dCIL::dListNode*>::dListNode* defNode = defsList.GetFirst(); defNode; defNode = defNode->GetNext()) {
//					dCIL::dListNode* const killStement = defNode->GetInfo();
//					if (killStement != point.m_statement) {
//						point.m_killStmtSet.Insert(killStement);
//					}
//				}
//				break;
//			}


			//case dThreeAdressStmt::m_enter:
			//case dThreeAdressStmt::m_leave:
			//case dThreeAdressStmt::m_load:
			case dThreeAdressStmt::m_store:
			case dThreeAdressStmt::m_storeBase:
			//case dThreeAdressStmt::m_push:
			//case dThreeAdressStmt::m_pop:
			case dThreeAdressStmt::m_function:
			case dThreeAdressStmt::m_nop:
			case dThreeAdressStmt::m_if:
			case dThreeAdressStmt::m_ifnot:
			case dThreeAdressStmt::m_ret:
			case dThreeAdressStmt::m_goto:
			case dThreeAdressStmt::m_label:
			case dThreeAdressStmt::m_argument:
				break;

			default:
				dAssert (0);
		}
*/
	}
}


void dDataFlowGraph::TraceReachIn (dCIL::dListNode* const node) const
{
	dCILInstr* const stmt = node->GetInfo();	
	stmt->Trace();
	const dDataFlowPoint& point = m_dataFlowGraph.Find(node)->GetInfo();
	if (point.m_reachStmtInputSet.GetCount()) {
		dTrace(("\t\t\treached by:\n"));

		dDataFlowPoint::dVariableSet<dCIL::dListNode*>::Iterator iter(point.m_reachStmtInputSet);
		for (iter.Begin(); iter; iter++) {
			dCIL::dListNode* const reachNode = iter.GetKey();
			dCILInstr* const reachInstr = reachNode->GetInfo();
			dTrace(("\t\t\t"));
			reachInstr->Trace();
		}
		dTrace(("\n"));
	}
}

void dDataFlowGraph::TraceReachOutput(dCIL::dListNode* const node) const
{
	dCILInstr* const stmt = node->GetInfo();
	stmt->Trace();
	const dDataFlowPoint& point = m_dataFlowGraph.Find(node)->GetInfo();
	if (point.m_reachStmtInputSet.GetCount()) {
		dTrace(("\t\t\treach to:\n"));

		dDataFlowPoint::dVariableSet<dCIL::dListNode*>::Iterator iter(point.m_reachStmtOutputSet);
		for (iter.Begin(); iter; iter++) {
			dCIL::dListNode* const reachedNode = iter.GetKey();
			dCILInstr* const reachedInstr = reachedNode->GetInfo();
			dTrace(("\t\t\t"));
			reachedInstr->Trace();
		}
		dTrace(("\n"));
	}
}


void dDataFlowGraph::CalculateReachingDefinitions()
{
	BuildDefinedAndKilledStatementSets ();

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

			//int test = 1;
			//for (dCIL::dListNode* stmtNode = block->m_begin; test; stmtNode = stmtNode->GetNext()) {
			for (dCIL::dListNode* node = block->m_begin; node != block->m_end; node = node->GetNext()) {
				//test = (stmtNode != block->m_end);
				dAssert (m_dataFlowGraph.Find(node));
				dDataFlowPoint& info = m_dataFlowGraph.Find(node)->GetInfo();

				//DTRACE_INTRUCTION (&info.m_statement->GetInfo());		

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


#if 0
	for (dCIL::dListNode* node = m_basicBlocks.m_begin; node != m_basicBlocks.m_end; node = node->GetNext()) {
		TraceReachIn (node);
	}
#endif

#if 0
	for (dCIL::dListNode* node = m_basicBlocks.m_begin; node != m_basicBlocks.m_end; node = node->GetNext()) {
		TraceReachOutput (node);
	}
#endif

}

void dDataFlowGraph::UpdateLiveInputLiveOutput()
{
	CalculateLiveInputLiveOutput ();
}

void dDataFlowGraph::UpdateReachingDefinitions()
{
	// for now just brute force regeneration
	CalculateReachingDefinitions();
}


void dDataFlowGraph::BuildGeneratedAndUsedVariableSets()
{
	dTree<dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();
		point.m_usedVariableSet.RemoveAll();
	}

	m_variableUsed.RemoveAll();
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();
		point.m_generatedVariable.Empty();
		dCILInstr* const instruc = point.m_statement->GetInfo();
//instruc->Trace();
		instruc->AddUsedVariable (m_variableUsed);
		instruc->AddGeneratedAndUsedSymbols(point);	

/*
		dThreeAdressStmt& stmt = point.m_statement->GetInfo();	
//DTRACE_INTRUCTION (&stmt);		

		point.m_generatedVariable.Empty();
		switch (stmt.m_instruction)
		{
			case dThreeAdressStmt::m_argument:
			{
				point.m_generatedVariable = stmt.m_arg0.m_label;
				break;
			}

			case dThreeAdressStmt::m_load:
			{
				dAssert (0);
				point.m_generatedVariable = stmt.m_arg0.m_label;
				point.m_usedVariableSet.Insert(stmt.m_arg1.m_label);
				point.m_usedVariableSet.Insert(stmt.m_arg2.m_label);
				break;
			}

			case dThreeAdressStmt::m_store:
			{
				dAssert (0);
				point.m_usedVariableSet.Insert(stmt.m_arg0.m_label);
				point.m_usedVariableSet.Insert(stmt.m_arg1.m_label);
				point.m_usedVariableSet.Insert(stmt.m_arg2.m_label);
				break;
			}


			case dThreeAdressStmt::m_new:
			{
				point.m_generatedVariable = stmt.m_arg0.m_label;
				point.m_usedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}

			case dThreeAdressStmt::m_release:
			{
				point.m_usedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}

			case dThreeAdressStmt::m_ifnot:
			{
				if (stmt.m_arg0.GetType().m_intrinsicType == dThreeAdressStmt::m_int) {
					point.m_usedVariableSet.Insert(stmt.m_arg0.m_label);
				} else {
					dAssert (0);
				}
				break;
			}

//			case dThreeAdressStmt::m_push:
//			{
//				point.m_usedVariableSet.Insert(stmt.m_arg0.m_label);
//				break;
//			}


//			case dThreeAdressStmt::m_pop:
			case dThreeAdressStmt::m_nop:
			case dThreeAdressStmt::m_goto:
			case dThreeAdressStmt::m_label:
			//case dThreeAdressStmt::m_enter:
			//case dThreeAdressStmt::m_leave:
			case dThreeAdressStmt::m_function:
				break;

			default:
				dAssert (0);
		}
*/
	}
}


void dDataFlowGraph::CalculateLiveInputLiveOutput ()
{
	BuildGeneratedAndUsedVariableSets();

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
			for (dCIL::dListNode* node = block->m_end->GetPrev(); test; node = node->GetPrev()) {
				test = (node != block->m_begin);

				dAssert (m_dataFlowGraph.Find(node));
				dDataFlowPoint& info = m_dataFlowGraph.Find(node)->GetInfo();
				dDataFlowPoint::dVariableSet<dString> oldInput (info.m_liveInputSet);
				dDataFlowPoint::dVariableSet<dString> oldOutput (info.m_liveOutputSet);

				info.m_liveInputSet.RemoveAll();
				info.m_liveInputSet.Union(info.m_liveOutputSet);
				if (info.m_generatedVariable.Size()) {
					info.m_liveInputSet.Remove(info.m_generatedVariable);
				}
				info.m_liveInputSet.Union(info.m_usedVariableSet);

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

void dDataFlowGraph::FindNodesInPathway(dCIL::dListNode* const source, dCIL::dListNode* const destination, dTree<int, dCIL::dListNode*>& pathOut) const
{
	m_mark ++;
	dList<dDataFlowPoint*> queue; 
	queue.Append(&m_dataFlowGraph.Find(source)->GetInfo());
	dAssert (queue.GetFirst()->GetInfo()->m_statement == source);

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
//	pathOut.Remove(source);
}


//bool dDataFlowGraph::DoStatementAreachesStatementB(dCIL::dListNode* const stmtNodeB, dCIL::dListNode* const stmtNodeA) const
bool dDataFlowGraph::DoMoveReachInstruction(dCIL::dListNode* const stmtNodeB, dCILInstrMove* const move) const
{
	bool canApplyPropagation = false;	

	dDataFlowPoint& info = m_dataFlowGraph.Find(stmtNodeB)->GetInfo();
	dDataFlowPoint::dVariableSet<dCIL::dListNode*>& reachingInputs = info.m_reachStmtInputSet;

//	if (reachingInputs.Find (stmtNodeA)) {
	if (reachingInputs.Find (move->GetNode())) {
		canApplyPropagation = true;

		//const dThreeAdressStmt& constStmt = stmtNodeA->GetInfo();
		//dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const definitions = m_variableDefinitions.Find (constStmt.m_arg0.m_label);
		dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const definitions = m_variableDefinitions.Find (move->GetArg0().m_label);
		dAssert (definitions);

		dList<dCIL::dListNode*>& defintionList = definitions->GetInfo();
		for (dList<dCIL::dListNode*>::dListNode* otherStmtNode = defintionList.GetFirst(); otherStmtNode; otherStmtNode = otherStmtNode->GetNext()){
			dCIL::dListNode* const duplicateDefinition = otherStmtNode->GetInfo();
			//if (duplicateDefinition != stmtNodeA) {
			if (duplicateDefinition != move->GetNode()) {
				if (reachingInputs.Find(duplicateDefinition)) {
					return false;
				}
			}
		}
		
		dTree<dList<dCIL::dListNode*>, dString>::dTreeNode* const definedStatements = m_variableDefinitions.Find(move->GetArg1().m_label);
		if (definedStatements) {
			dTree<int, dCIL::dListNode*> path;
			FindNodesInPathway(move->GetNode(), stmtNodeB, path);

			dList<dCIL::dListNode*>& statementList = definedStatements->GetInfo();
			for (dList<dCIL::dListNode*>::dListNode* ptr = statementList.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dCIL::dListNode* const stmt = ptr->GetInfo();
				if (path.Find(stmt)) {
					return false;
				}
			}
		}
	}

	return canApplyPropagation;
}


bool dDataFlowGraph::ApplyCopyPropagation()
{
	bool ret = false;
	UpdateReachingDefinitions();
//m_cil->Trace();

	for (dCIL::dListNode* node = m_basicBlocks.m_begin; node != m_basicBlocks.m_end; node = node->GetNext()) {
		dCILInstrMove* const moveInst = node->GetInfo()->GetAsMove();
		if (moveInst) {
moveInst->Trace();
			dInstructionVariableDictionary::dTreeNode* const usedVariableNode = m_variableUsed.Find (moveInst->GetArg0().m_label);
			if (usedVariableNode) {
				const dList<dCIL::dListNode*>& usedVariableList = usedVariableNode->GetInfo();
				for (const dList<dCIL::dListNode*>::dListNode* useVariableNode = usedVariableList.GetFirst() ; useVariableNode; useVariableNode = useVariableNode->GetNext()) {
					dCIL::dListNode* const node1 = useVariableNode->GetInfo();
					dCILInstr* const inst = node1->GetInfo();
					dTrace (("\t\t"));
					inst->Trace();
					//TraceReachIn (node1);

					if (DoMoveReachInstruction(node1, moveInst)) {
						ret |= inst->ApplyCopyPropagation(moveInst);
					}
				}
			}

m_cil->Trace();
		}
	}

	return ret;
}



bool dDataFlowGraph::ApplyRemoveDeadCode()
{
	CalculateLiveInputLiveOutput ();
	bool ret = false;
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = m_basicBlocks.m_begin; node != m_basicBlocks.m_end; node = nextNode) {
		nextNode = node->GetNext();
		dCILInstr* const instr = node->GetInfo();
		//instr->Trace();
		ret |= instr->ApplyDeadElimination (*this);
	}
	return ret;
}

bool dDataFlowGraph::RemoveDeadInstructions()
{
	bool ret = false;

	int count = 0;
	for (dCIL::dListNode* node = m_basicBlocks.m_begin; (node != m_basicBlocks.m_end) && (node->GetInfo()->GetAsNop() || node->GetInfo()->GetAsArgument()) && (count < D_CALLER_SAVE_REGISTER_COUNT); node = node->GetNext()) {
		//dThreeAdressStmt& stmt = node->GetInfo();
		//dCILInstrArgument* const argInstr = node->GetInfo()->GetAsArgument();
		//stmt.m_instruction = dThreeAdressStmt::m_nop;
		if (node->GetInfo()->GetAsArgument()) {
			node->GetInfo()->Nullify();
			count++;
		}
	}

/*
	for (dCIL::dListNode* stmtNode = m_basicBlocks.m_begin->GetNext()->GetNext(); stmtNode != m_basicBlocks.m_end; stmtNode = stmtNode->GetNext()) {
		dThreeAdressStmt& stmt = stmtNode->GetInfo();
		if (stmt.m_instruction == dThreeAdressStmt::m_call) {
			int count = 0;
			for (dCIL::dListNode* node = stmtNode->GetPrev(); (node->GetInfo().m_instruction == dThreeAdressStmt::m_param) && (count < D_CALLER_SAVE_REGISTER_COUNT); node = node->GetPrev()) {
				dThreeAdressStmt& stmt1 = node->GetInfo();
				stmt1.m_instruction = dThreeAdressStmt::m_nop;
				count ++;
			}
		}
	}
*/
	return ret;
}

bool dDataFlowGraph::RemoveNop()
{
	bool ret = false;
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = m_basicBlocks.m_begin; node != m_basicBlocks.m_end; node = nextNode) {
		nextNode = node->GetNext();
		//dThreeAdressStmt& stmt = stmtNode->GetInfo();	
		//if (stmt.m_instruction == dThreeAdressStmt::m_nop) {
		if (node->GetInfo()->GetAsNop()) {
			//m_cil->Remove(stmtNode);
			delete node->GetInfo();
			ret = true;
		}
	}
	return ret;
}




void dDataFlowGraph::RegistersAllocation (int registerCount)
{
	dRegisterInterferenceGraph interferenceGraph(this, registerCount);
}

#endif




/*




*/

