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
#include "dRegisterInterferenceGraph.h"


#define D_SPILL_WEIGHT_FACTOR	10
#define D_MOVE_WEIGHT_FACTOR	20

dRegisterInterferenceGraph::dRegisterInterferenceGraph (dDataFlowGraph* const flowGraph, int registerCount)
	:dTree<dRegisterInterferenceNode, dString>()
	,m_flowGraph(flowGraph)
	,m_spillCount(0)
	,m_registerCount(registerCount)
	,m_spillPenatryFactor(0)
{
	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
	while (m_flowGraph->ApplyRemoveDeadCode());

	Build();
	while (ColorGraph () > m_registerCount) {
		// we have a spill, find a good spill node and try graph coloring again
		SelectSpillVariableAndReWriteFunction();
	}

	AllocateRegisters();
m_flowGraph->m_cil->Trace();

	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
	for (bool optimized = true; optimized;) {
		optimized = false;
		m_flowGraph->UpdateReachingDefinitions();
		optimized |= m_flowGraph->ApplyCopyPropagation();
m_flowGraph->m_cil->Trace();
		optimized |= m_flowGraph->ApplyRemoveDeadCode();
m_flowGraph->m_cil->Trace();
	}
}

void dRegisterInterferenceGraph::AllocateRegisters ()
{
	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();	
//stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_assigment:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						stmt.m_arg2.m_label = GetRegisterName (stmt.m_arg2.m_label);
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_free:
			case dTreeAdressStmt::m_loadBase:
			case dTreeAdressStmt::m_storeBase:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				break;
			}


			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_store:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg2.m_label = GetRegisterName (stmt.m_arg2.m_label);
				} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}
				break;
			}


			case dTreeAdressStmt::m_alloc:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
				break;
			}


			case dTreeAdressStmt::m_if:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
				}
				break;
			}

			case dTreeAdressStmt::m_enter:
			{
				int returnRegisterMask = ~((m_flowGraph->m_returnType == 1) ? (1 << D_RETURN_REGISTER_INDEX) : 0);
				int regMask = m_flowGraph->m_registersUsedMask & returnRegisterMask; 
				stmt.m_extraInformation = regMask;
				break;
			}

			case dTreeAdressStmt::m_leave:
			{
				int returnRegisterMask = ~((m_flowGraph->m_returnType == 1) ? (1 << D_RETURN_REGISTER_INDEX) : 0);
				int regMask = m_flowGraph->m_registersUsedMask & returnRegisterMask; 
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


int dRegisterInterferenceGraph::GetRegisterIndex (const dString& varName) const
{
	int index = -1;
	if (varName[0] != '_') {
		dTreeNode* const node = Find (varName);
		dRegisterInterferenceNode& var = node->GetInfo();
		index = var.m_registerIndex;
	}
	return index;
}



dString dRegisterInterferenceGraph::GetRegisterName (const dString& varName) const
{
	int index = GetRegisterIndex (varName);
	if (index == -1) {
		return varName;
	} else {
		return IndexToRegister(index);
	}
}

bool dRegisterInterferenceGraph::IsTempVariable (const dString& name) const
{
    int spillSize = strlen (D_SPILL_REGISTER_SYMBOL);
    return (name[0] != '_') && strncmp (name.GetStr(), D_SPILL_REGISTER_SYMBOL, spillSize);
}

void dRegisterInterferenceGraph::Build()
{
	dTree<dDataFlowGraph::dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_flowGraph->m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowGraph::dDataFlowPoint& point = iter.GetNode()->GetInfo();

		dDataFlowGraph::dDataFlowPoint::dVariableSet<dString>::Iterator definedIter (point.m_usedVariableSet);
		for (definedIter.Begin(); definedIter; definedIter ++) {
			const dString& variable = definedIter.GetKey();
			if (IsTempVariable(variable)) {
				dTreeNode* interferanceGraphNode = Find(variable);
				if (!interferanceGraphNode) {
					interferanceGraphNode = Insert(variable);
					interferanceGraphNode->GetInfo().m_name = variable;
//dTrace (("%s\n", variable.GetStr()));
				}
			}
		}
	}

	// pre-color some special nodes
	dTreeNode* const returnRegNode = Find(GetReturnVariableName());
	if (returnRegNode) {
		dRegisterInterferenceNode& returnReginster = returnRegNode->GetInfo();
		returnReginster.m_registerIndex = D_RETURN_REGISTER_INDEX;
		m_flowGraph->m_returnVariableName = GetRegisterName (returnReginster.m_name);
	}

	for (iter.Begin(); iter; iter ++) {
		dDataFlowGraph::dDataFlowPoint& info = iter.GetNode()->GetInfo();
		if (info.m_generatedVariable.Size()) {
			const dString& variableA = info.m_generatedVariable;

            if (IsTempVariable(variableA)) {
				dTreeNode* const nodeA = Find(variableA);
				dAssert (nodeA);
				dRegisterInterferenceNode& inteferanceNodeA = nodeA->GetInfo();
				dDataFlowGraph::dDataFlowPoint::dVariableSet<dString>::Iterator aliveIter (info.m_liveOutputSet);

				for (aliveIter.Begin(); aliveIter; aliveIter ++) {
					const dString& variableB = aliveIter.GetKey();
					if (IsTempVariable(variableB) && (variableA != variableB)) {
						dTreeNode* const nodeB = Find(variableB);
						dAssert (nodeB);
						dRegisterInterferenceNode& inteferanceNodeB = nodeB->GetInfo();

						bool hasEdge = false;
						for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = inteferanceNodeA.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
							dList<dRegisterInterferenceNodeEdge>::dListNode* const twinEdgeNode = ptr->GetInfo().m_twin;
							if (twinEdgeNode && (twinEdgeNode->GetInfo().m_incidentNode == nodeB)) {
								hasEdge = true;
								break;
							}
						}
						if (!hasEdge) {
							dList<dRegisterInterferenceNodeEdge>::dListNode* const entryA = inteferanceNodeA.m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeA));
							dList<dRegisterInterferenceNodeEdge>::dListNode* const entryB = inteferanceNodeB.m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeB));
							entryA->GetInfo().m_twin = entryB;
							entryB->GetInfo().m_twin = entryA;
						}
					}
				}
			}
		}
	}

//m_flowGraph->m_cil->Trace();

	dList<dTreeNode*> moveNodes;
	for (iter.Begin(); iter; iter ++) {
		dDataFlowGraph::dDataFlowPoint& info = iter.GetNode()->GetInfo();
		dTreeAdressStmt& stmt = info.m_statement->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_assigment) && (stmt.m_operator == dTreeAdressStmt::m_nothing) && (stmt.m_arg1.m_type != dTreeAdressStmt::m_intConst) &&  (stmt.m_arg1.m_type != dTreeAdressStmt::m_floatConst)) {

//stmt.Trace();
			const dString& variableA = stmt.m_arg0.m_label;
			const dString& variableB = stmt.m_arg1.m_label;
			dTreeNode* const nodeA = Find(variableA);
			dTreeNode* const nodeB = Find(variableB);

			bool canCoalese = true;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = nodeA->GetInfo().m_interferanceEdge.GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				dList<dRegisterInterferenceNodeEdge>::dListNode* const twinEdgeNode = edge.m_twin;
				if (twinEdgeNode->GetInfo().m_incidentNode == nodeB) {
					canCoalese = false;
					break;
				}
			}
			if (canCoalese) {
				nodeA->GetInfo().m_isMove = true;
				nodeB->GetInfo().m_isMove = true;
				if (nodeB == returnRegNode) {
					m_coalescedNodes.Append(dCoalescedNodePair (nodeB, nodeA));
				} else {
					m_coalescedNodes.Append(dCoalescedNodePair (nodeA, nodeB));
				}
			}
		}
	}
}

dRegisterInterferenceGraph::dTreeNode* dRegisterInterferenceGraph::GetBestNode()
{
	int minCount = 0x7fffff;
	dTreeNode* bestNode = NULL;
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dRegisterInterferenceNode& info = node->GetInfo();
		if (!info.m_inSet) {
			int count = info.m_isMove ? 4096 : 0;
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
}

bool dRegisterInterferenceGraph::CoalesceNodesRule1(dTreeNode* const nodePairA, dTreeNode* const nodePairB)
{
	dTree<dTreeNode*, dTreeNode*> edgeUnion;
	dRegisterInterferenceNode& nodeA = nodePairA->GetInfo();
	dList<dRegisterInterferenceNodeEdge>& edgeListInfoA = nodeA.m_interferanceEdge; 
	for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoA.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
		dTreeNode* const otherNode = edge.m_twin->GetInfo().m_incidentNode;
		if (!otherNode->GetInfo().m_inSet) {
			edgeUnion.Insert(otherNode, otherNode);
		}
	}

	dRegisterInterferenceNode& nodeB = nodePairB->GetInfo();
	dList<dRegisterInterferenceNodeEdge>& edgeListInfoB = nodeB.m_interferanceEdge; 
	for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoB.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
		dTreeNode* const otherNode = edge.m_twin->GetInfo().m_incidentNode;
		if (!otherNode->GetInfo().m_inSet) {
			edgeUnion.Insert(otherNode, otherNode);
		}
	}

	if (edgeUnion.GetCount() < m_registerCount) {
		nodeB.m_registerIndex = nodeA.m_registerIndex;
		nodeB.m_coalescedParent = nodePairA;

		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoB.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			edge.m_mark = true;
			edge.m_twin->GetInfo().m_mark = true;
		}

		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoA.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			dTreeNode* const otherNode = edge.m_twin->GetInfo().m_incidentNode;
			edgeUnion.Remove(otherNode);
		}

		dTree<dTreeNode*, dTreeNode*>::Iterator iter (edgeUnion);
		for (iter.Begin(); iter; iter ++) {
			dTreeNode* const nodeA = nodePairA;
			dTreeNode* const nodeB = iter.GetNode()->GetInfo();

			dList<dRegisterInterferenceNodeEdge>::dListNode* const entryA = nodeA->GetInfo().m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeA));
			dList<dRegisterInterferenceNodeEdge>::dListNode* const entryB = nodeB->GetInfo().m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeB));
			entryA->GetInfo().m_twin = entryB;
			entryB->GetInfo().m_twin = entryA;
		}
		return true;
	}
	return false;

}

bool dRegisterInterferenceGraph::CoalesceNodesRule2(dTreeNode* const nodeA, dTreeNode* const nodeB)
{
	return false;
}


void dRegisterInterferenceGraph::CoalesceNodes()
{
	for (bool hasCoalesced = true; hasCoalesced; ) {
		hasCoalesced = false;
		dList<dCoalescedNodePair>::dListNode* nextNode;
		for (dList<dCoalescedNodePair>::dListNode* coalsescedNodePair = m_coalescedNodes.GetFirst(); coalsescedNodePair; coalsescedNodePair = nextNode) {
			const dCoalescedNodePair& pair = coalsescedNodePair->GetInfo();
			nextNode = coalsescedNodePair->GetNext();

			bool coalesced = false;
			dRegisterInterferenceNode& nodeA = pair.m_nodeA->GetInfo();
			dList<dRegisterInterferenceNodeEdge>& edgeListInfoA = nodeA.m_interferanceEdge; 
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoA.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
				dTreeNode* const otherNode = edge.m_twin->GetInfo().m_incidentNode;
				if (pair.m_nodeB == otherNode) {
					coalesced = true;
					break;
				}
			}

			coalesced = coalesced || CoalesceNodesRule1 (pair.m_nodeA, pair.m_nodeB) || CoalesceNodesRule2 (pair.m_nodeA, pair.m_nodeB);
			if (coalesced) {
				m_coalescedNodes.Remove(coalsescedNodePair);
			}
			hasCoalesced |= coalesced;
		}
	}
}

int dRegisterInterferenceGraph::ColorGraph ()
{
	dList<dTreeNode*> registerOrder;

	m_flowGraph->m_registersUsedMask = 0;
	for (int i = 0; i < GetCount(); i ++) {
		CoalesceNodes();

		dTreeNode* const bestNode = GetBestNode();
		registerOrder.Addtop(bestNode);
		dRegisterInterferenceNode& node = bestNode->GetInfo();
		dList<dRegisterInterferenceNodeEdge>& edgeListInfo = node.m_interferanceEdge; 
		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfo.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			edge.m_mark = true;
			edge.m_twin->GetInfo().m_mark = true;
		}

		if (node.m_isMove) {
			dList<dCoalescedNodePair>::dListNode* nextNode;
			for (dList<dCoalescedNodePair>::dListNode* coalsescedNodePair = m_coalescedNodes.GetFirst(); coalsescedNodePair; coalsescedNodePair = nextNode) {
				const dCoalescedNodePair& pair = coalsescedNodePair->GetInfo();
				nextNode = coalsescedNodePair->GetNext();
				if ((bestNode == pair.m_nodeA) || (bestNode == pair.m_nodeB)) {
					m_coalescedNodes.Remove(coalsescedNodePair);
				}
			}
		}
	} 

    for (dList<dTreeNode*>::dListNode* node = registerOrder.GetFirst(); node; node = node->GetNext()) {
        dTreeNode* const varNode = node->GetInfo();
        dRegisterInterferenceNode& variable = varNode->GetInfo();
        if ((variable.m_registerIndex != -1) && (variable.m_coalescedParent)) {
            dRegisterInterferenceNode& parentVariable = variable.m_coalescedParent->GetInfo();
            variable.m_registerIndex = parentVariable.m_registerIndex;
        }
    }

	int registersUsed = -1;
	for (dList<dTreeNode*>::dListNode* node = registerOrder.GetFirst(); node; node = node->GetNext()) {
		dTreeNode* const varNode = node->GetInfo();
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		if (variable.m_registerIndex == -1) {
			dTreeNode* const parentNode = (dTreeNode*)variable.m_coalescedParent;

			if (variable.m_coalescedParent && (variable.m_coalescedParent->GetInfo().m_registerIndex != -1)) {
				dRegisterInterferenceNode& parentVariable = variable.m_coalescedParent->GetInfo();
				variable.m_registerIndex = parentVariable.m_registerIndex;
			}  else {
				int regMask = 0;
				for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = variable.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
					dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
					dRegisterInterferenceNode& otherVariable = edge.m_twin->GetInfo().m_incidentNode->GetInfo();
					if (otherVariable.m_registerIndex != -1) {
						regMask |= (1 << otherVariable.m_registerIndex);
					}
				}

				int index = 0;
				for (int mask = regMask; mask & 1 ; mask >>= 1) {
					index ++;
				}
				variable.m_registerIndex = index;

				if (variable.m_coalescedParent) {
					dRegisterInterferenceNode& parentVariable = variable.m_coalescedParent->GetInfo();
					parentVariable.m_registerIndex = variable.m_registerIndex;
				}
			}
		}
		m_flowGraph->m_registersUsedMask |= (1 << variable.m_registerIndex);
		registersUsed = dMax (registersUsed, variable.m_registerIndex);
	}

	return registersUsed + 1;
}


void dRegisterInterferenceGraph::SelectSpillVariableAndReWriteFunction()
{
	dTree<dVariableSpillPriority,dString> spillPriority;
	dTree<dDataFlowGraph::dDataFlowPoint, dCIL::dListNode*>::Iterator stmtIter (m_flowGraph->m_dataFlowGraph);

	for (stmtIter.Begin(); stmtIter; stmtIter ++) {
		dDataFlowGraph::dDataFlowPoint& point = stmtIter.GetNode()->GetInfo();
//		dTreeAdressStmt& stmt = point.m_statement->GetInfo();	
//stmt.Trace();
        if (point.m_generatedVariable.GetStr()) {
            dTree<dVariableSpillPriority,dString>::dTreeNode* node = spillPriority.Find(point.m_generatedVariable);
            if (!node) {
                node = spillPriority.Insert(point.m_generatedVariable);
            }
            node->GetInfo().m_useCount += 1;
        }

        dDataFlowGraph::dDataFlowPoint::dVariableSet<dString>::Iterator usedIter (point.m_usedVariableSet);
        for (usedIter.Begin(); usedIter; usedIter ++) {
            const dString& key = usedIter.GetKey();
            dTree<dVariableSpillPriority,dString>::dTreeNode* node = spillPriority.Find(key);
            if (!node) {
                node = spillPriority.Insert(key);
            }
            node->GetInfo().m_useCount += 1;
        }
	}

	dList<dDataFlowGraph::dLoop> loops;
	m_flowGraph->GetLoops (loops);
	for (dList<dDataFlowGraph::dLoop>::dListNode* loopNode = loops.GetFirst(); loopNode; loopNode = loopNode->GetNext()) {
		dDataFlowGraph::dLoop& loop = loopNode->GetInfo();

		dCIL::dListNode* node = loop.m_head;
		do {

			dTreeAdressStmt& stmt = node->GetInfo();	
//stmt.Trace();
			switch (stmt.m_instruction)
			{

				case dTreeAdressStmt::m_assigment:
				{
					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg0.m_label);
					if (priorityNode) {
						priorityNode->GetInfo().m_useCount --;
						priorityNode->GetInfo().m_loopUseCount ++;
					}

					if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
						dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg1.m_label);
						if (priorityNode) {
							priorityNode->GetInfo().m_useCount --;
							priorityNode->GetInfo().m_loopUseCount ++;
						}
					} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}

					if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
						if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
							dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg2.m_label);
							if (priorityNode) {
								priorityNode->GetInfo().m_useCount --;
								priorityNode->GetInfo().m_loopUseCount ++;
							}
						} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
							dAssert (0);
						}
					}
					break;
				}

				case dTreeAdressStmt::m_if:
				{
					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode0 = spillPriority.Find(stmt.m_arg0.m_label);
					if (priorityNode0) {
						priorityNode0->GetInfo().m_useCount --;
						priorityNode0->GetInfo().m_loopUseCount ++;
					}

					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode1 = spillPriority.Find(stmt.m_arg1.m_label);
					if (priorityNode1) {
						priorityNode1->GetInfo().m_useCount --;
						priorityNode1->GetInfo().m_loopUseCount ++;
					}
					break;
				}

				case dTreeAdressStmt::m_pop:
				case dTreeAdressStmt::m_push:
				case dTreeAdressStmt::m_storeBase:
				case dTreeAdressStmt::m_loadBase:
				{
					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode0 = spillPriority.Find(stmt.m_arg0.m_label);
					if (priorityNode0) {
						priorityNode0->GetInfo().m_useCount --;
						priorityNode0->GetInfo().m_loopUseCount ++;
					}
					break;
				}
                
   				case dTreeAdressStmt::m_load:
                case dTreeAdressStmt::m_store:
                {
                    dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode0 = spillPriority.Find(stmt.m_arg0.m_label);
                    if (priorityNode0) {
                        priorityNode0->GetInfo().m_useCount --;
                        priorityNode0->GetInfo().m_loopUseCount ++;
                    }

                    dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode1 = spillPriority.Find(stmt.m_arg1.m_label);
                    if (priorityNode1) {
                        priorityNode1->GetInfo().m_useCount --;
                        priorityNode1->GetInfo().m_loopUseCount ++;
                    }

                    dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode2 = spillPriority.Find(stmt.m_arg2.m_label);
                    if (priorityNode2) {
                        priorityNode2->GetInfo().m_useCount --;
                        priorityNode2->GetInfo().m_loopUseCount ++;
                    }
   					break;
                }

/*
				case dTreeAdressStmt::m_free:


				case dTreeAdressStmt::m_alloc:
				{
					stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
					stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
					break;
				}


*/
				case dTreeAdressStmt::m_call:
				case dTreeAdressStmt::m_goto:
				case dTreeAdressStmt::m_label:
				case dTreeAdressStmt::m_nop:
				case dTreeAdressStmt::m_enter:
				case dTreeAdressStmt::m_leave:
				case dTreeAdressStmt::m_function:
				case dTreeAdressStmt::m_argument:
					break;

				default:
				dAssert (0);
			}
			node = node->GetNext();
		} while (node!= loop.m_tail);
	}


	dFloat lowestUsage = 1.0e10f;
	dTree<dVariableSpillPriority,dString>::dTreeNode* spillCandidate = NULL;
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		const dString& variable = iter.GetNode()->GetKey();
		dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(variable);
		dAssert (priorityNode);

		dTree<int, dString>::dTreeNode* const spillPenaltyNode = m_spillPenalty.Find(variable);
		int penalty = spillPenaltyNode ? spillPenaltyNode->GetInfo() : 0;

		dRegisterInterferenceNode& iterferanceInfo = iter.GetNode()->GetInfo();
		dList<dRegisterInterferenceNodeEdge>& edgeListInfo = iterferanceInfo.m_interferanceEdge; 
		dFloat priority = dFloat ((iterferanceInfo.m_isMove ? D_MOVE_WEIGHT_FACTOR : 0) + penalty + priorityNode->GetInfo().m_useCount + D_SPILL_WEIGHT_FACTOR * priorityNode->GetInfo().m_loopUseCount) / (edgeListInfo.GetCount() + 1);
		if (priority < lowestUsage) {
			lowestUsage = priority;
			spillCandidate = priorityNode;
		}
	}

	dAssert (spillCandidate);
	ReWriteFunctionSpillingVarible (spillCandidate->GetKey());
}


void dRegisterInterferenceGraph::SaveSpillRegister(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument, const dString& spillVariable, const dString& spillMemory)
{
	if (argument.m_label == spillVariable) {
		dString newtemp = m_flowGraph->m_cil->NewTemp (); 
		argument.m_label = newtemp;

		dTree<int, dString>::dTreeNode* penaltyNode = m_spillPenalty.Find(newtemp);
		if (penaltyNode) {
			penaltyNode->GetInfo() = m_spillPenatryFactor;
		} else {
			m_spillPenalty.Insert(m_spillPenatryFactor, newtemp);
		}

		dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
		m_flowGraph->m_cil->InsertAfter (node, newNode);

		dTreeAdressStmt& stmt = newNode->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_storeBase;
		stmt.m_operator = dTreeAdressStmt::m_nothing;
		stmt.m_arg0.m_label = newtemp;
		stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
		stmt.m_arg2.m_label = spillMemory;
		stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;

//node->GetInfo().Trace();
//stmt.Trace();
	}
}

void dRegisterInterferenceGraph::LoadSpillRegister(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument, const dString& spillVariable, const dString& spillMemory)
{
	if (argument.m_label == spillVariable) {
		dString newtemp = m_flowGraph->m_cil->NewTemp (); 
		argument.m_label = newtemp;

		dTree<int, dString>::dTreeNode* penaltyNode = m_spillPenalty.Find(newtemp);
		if (penaltyNode) {
			penaltyNode->GetInfo() = m_spillPenatryFactor;
		} else {
			m_spillPenalty.Insert(m_spillPenatryFactor, newtemp);
		}


		dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
		m_flowGraph->m_cil->InsertAfter (node->GetPrev(), newNode);

		dTreeAdressStmt& stmt = newNode->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_loadBase;
		stmt.m_operator = dTreeAdressStmt::m_nothing;
		stmt.m_arg0.m_label = newtemp;
		stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
		stmt.m_arg2.m_label = spillMemory;
		stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;

//stmt.Trace();
//node->GetInfo().Trace();
	}
}


void dRegisterInterferenceGraph::ReWriteFunctionSpillingVarible(const dString& spillVariable)
{
//m_flowGraph->m_cil->Trace();

	dString spillLocal = IndexToLocal(m_spillCount);
	m_spillCount ++;

	dCIL::dListNode* nextNode;
	m_spillPenatryFactor += 50;
	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = nextNode) {
		nextNode = node->GetNext();
		dTreeAdressStmt& stmt = node->GetInfo();
//stmt.Trace();
		switch (stmt.m_instruction)
		{
            case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_loadBase:
			{
				SaveSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				LoadSpillRegister (node, stmt.m_arg1, spillVariable, spillLocal);
				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					LoadSpillRegister (node, stmt.m_arg2, spillVariable, spillLocal);
				}
				SaveSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				LoadSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
				LoadSpillRegister (node, stmt.m_arg1, spillVariable, spillLocal);
				break;
			}

            case dTreeAdressStmt::m_load:
            {
                LoadSpillRegister (node, stmt.m_arg2, spillVariable, spillLocal);
                LoadSpillRegister (node, stmt.m_arg1, spillVariable, spillLocal);
                SaveSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
            }

			case dTreeAdressStmt::m_store:
			{
                LoadSpillRegister (node, stmt.m_arg2, spillVariable, spillLocal);
                LoadSpillRegister (node, stmt.m_arg1, spillVariable, spillLocal);
                LoadSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
				break;
			}

            case dTreeAdressStmt::m_push:
            case dTreeAdressStmt::m_storeBase:
            {
               LoadSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
               break;
            }
/*
			
			case dTreeAdressStmt::m_alloc:
			{
				dAssert (0);
				//point.m_killVariable = stmt.m_arg0.m_label;
				//point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}

			case dTreeAdressStmt::m_free:
			{
				dAssert (0);
				//point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
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

//m_flowGraph->m_cil->Trace();
	m_flowGraph->m_returnVariableName = GetReturnVariableName();
	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
	RemoveAll();
	Build();
}
