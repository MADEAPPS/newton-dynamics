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
#include "dCILInstrMiscellaneous.h"
#include "dRegisterInterferenceGraph.h"



#define D_SPILL_WEIGHT_FACTOR	10
#define D_MOVE_WEIGHT_FACTOR	20

dRegisterInterferenceGraph::dRegisterInterferenceGraph (dDataFlowGraph* const flowGraph, int registerCount)
	:dTree<dRegisterInterferenceNode, dString>()
	,m_spillPenalty()
	,m_coalescedNodes()
	,m_flowGraph(flowGraph)
	,m_spillCount(0)
	,m_registerCount(registerCount)
	,m_spillPenatryFactor(0)
{
	m_flowGraph->ApplyInstructionSematicOrdering();
	m_flowGraph->BuildBasicBlockGraph();

	for (bool optimized = true; optimized;) {
		optimized = false;
//		m_flowGraph->CalculateLiveInputLiveOutput();
//		m_flowGraph->UpdateReachingDefinitions();
//		optimized |= m_flowGraph->ApplyCopyPropagation();
//m_flowGraph->m_cil->Trace();
		//m_flowGraph->CalculateLiveInputLiveOutput();
		optimized |= m_flowGraph->ApplyRemoveDeadCode();
//m_flowGraph->m_cil->Trace();
	}

	//m_flowGraph->CalculateLiveInputLiveOutput();
	Build();
	while (ColorGraph () > m_registerCount) {
		// we have a spill, find a good spill node and try graph coloring again
dAssert (0);
		SelectSpillVariableAndReWriteFunction();
	}

	AllocateRegisters();
//m_flowGraph->m_cil->Trace();

	m_flowGraph->BuildBasicBlockGraph();
	for (bool optimized = true; optimized;) {
		optimized = false;
//		m_flowGraph->CalculateLiveInputLiveOutput();
//		m_flowGraph->UpdateReachingDefinitions();
//		optimized |= m_flowGraph->ApplyCopyPropagation();
//m_flowGraph->m_cil->Trace();
//		m_flowGraph->CalculateLiveInputLiveOutput();
//m_flowGraph->m_cil->Trace();
		optimized |= m_flowGraph->ApplyRemoveDeadCode();
//m_flowGraph->m_cil->Trace();
	}

//m_flowGraph->m_cil->Trace();	
	while (m_flowGraph->RemoveRedundantJumps());
//m_flowGraph->m_cil->Trace();

	m_flowGraph->ApplyRemoveDeadCode();
//m_flowGraph->m_cil->Trace();
	m_flowGraph->RemoveDeadInstructions();
//m_flowGraph->m_cil->Trace();

	m_flowGraph->RemoveNop();

	InsertEpilogAndProlog();
m_flowGraph->m_cil->Trace();
}

void dRegisterInterferenceGraph::AllocateRegisters ()
{
	for (dCIL::dListNode* node = m_flowGraph->m_basicBlocks.m_begin; node != m_flowGraph->m_basicBlocks.m_end; node = node->GetNext()) {
		dCILInstr* const instr = node->GetInfo();
//instr->Trace();
		instr->AsignRegisterName(*this);
//instr->Trace();
	}
}


int dRegisterInterferenceGraph::GetRegisterIndex (const dString& varName) const
{
	int index = -1;
//	if (varName[0] != '_') {
		dTreeNode* const node = Find (varName);
		dRegisterInterferenceNode& var = node->GetInfo();
		index = var.m_registerIndex;
//	}
	return index;
}



dString dRegisterInterferenceGraph::GetRegisterName (const dString& varName) const
{
	int index = GetRegisterIndex (varName);
	if (index == -1) {
		dAssert (0);
		return varName;
	} else {
		return IndexToRegister(index);
	}
}

bool dRegisterInterferenceGraph::IsTempVariable (const dString& name) const
{
    int spillSize = strlen (D_SPILL_REGISTER_SYMBOL);
//    return (name[0] != '_') && strncmp (name.GetStr(), D_SPILL_REGISTER_SYMBOL, spillSize);
	return strncmp (name.GetStr(), D_SPILL_REGISTER_SYMBOL, spillSize);
}

void dRegisterInterferenceGraph::Build()
{
	m_flowGraph->CalculateLiveInputLiveOutput();

	dTree<dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_flowGraph->m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& point = iter.GetNode()->GetInfo();

		dDataFlowPoint::dVariableSet<dString>::Iterator definedIter (point.m_usedVariableSet);
		for (definedIter.Begin(); definedIter; definedIter ++) {
			const dString& variable = definedIter.GetKey();
			if (IsTempVariable(variable)) {
				dTreeNode* interferanceGraphNode = Find(variable);
				if (!interferanceGraphNode) {
					interferanceGraphNode = Insert(variable);
					interferanceGraphNode->GetInfo().m_name = variable;
dTrace (("%s\n", variable.GetStr()));
				}
			}
		}
	}

	// pre-color some special nodes
	int intArgumentIndex = D_CALLER_SAVE_REGISTER_INDEX; 
	for (dCIL::dListNode* node = m_flowGraph->m_basicBlocks.m_begin; node != m_flowGraph->m_basicBlocks.m_end; node = node->GetNext()) {
		dCILInstr* const instr = node->GetInfo();
//instr->Trace();

		if (instr->GetAsArgument()) {
			dCILInstrArgument* const argInst = instr->GetAsArgument();
			const dCILInstr::dArg& arg = argInst->GetArg0();
			switch (arg.GetType().m_intrinsicType)
			{
				case dCILInstr::m_int:
				{
					if (intArgumentIndex < D_CALLER_SAVE_REGISTER_COUNT) {
						dTreeNode* const returnRegNode = Find(arg.m_label);
						dAssert(returnRegNode);
						dRegisterInterferenceNode& registerInfo = returnRegNode->GetInfo();
						registerInfo.m_registerIndex = intArgumentIndex;
						//registerInfo.m_isPrecolored = true;
						intArgumentIndex++;
					}
					else {
						dAssert(0);
					}
					break;
				}

				default:
					dAssert(0);
			}
		} else if (instr->GetAsReturn()) {
			dCILInstrReturn* const retInstr = instr->GetAsReturn();

			const dCILInstr::dArg& arg = retInstr->GetArg0();
			if (arg.GetType().m_isPointer || (arg.GetType().m_intrinsicType != dCILInstr::m_void)) {
				switch (arg.GetType().m_intrinsicType)
				{
					case dCILInstr::m_int:
					{
						dTreeNode* const returnRegNode = Find(arg.m_label);
						dAssert(returnRegNode);
						dRegisterInterferenceNode& registerInfo = returnRegNode->GetInfo();
						registerInfo.m_registerIndex = D_RETURN_REGISTER_INDEX;
						//registerInfo.m_isPrecolored = true;
						break;
					}
					default:
					{

					}
				}
			}

		} else if (instr->GetAsCall()) {
			dCILInstrCall* const callInstr = instr->GetAsCall();

			const dCILInstr::dArg& arg = callInstr->GetArg0();
			if (arg.GetType().m_isPointer || (arg.GetType().m_intrinsicType != dCILInstr::m_void)) {
				switch (arg.GetType().m_intrinsicType)
				{
					case dCILInstr::m_int:
					{
						dTreeNode* const returnRegNode = Find (arg.m_label);
						dAssert(returnRegNode);
						dRegisterInterferenceNode& registerInfo = returnRegNode->GetInfo();
						registerInfo.m_registerIndex = D_RETURN_REGISTER_INDEX;
						//registerInfo.m_isPrecolored = true;
						break;
					}
					default:
						dAssert(0);
				}
			}
			
			int index = D_CALLER_SAVE_REGISTER_INDEX;
			for (dList<dCILInstr::dArg>::dListNode* node = callInstr->m_parameters.GetLast(); node && (index < D_CALLER_SAVE_REGISTER_COUNT); node = node->GetPrev()) {
				const dCILInstr::dArg& arg = node->GetInfo();
				dTreeNode* const regNode = Find (arg.m_label);
				dAssert(regNode);
				dRegisterInterferenceNode& registerInfo = regNode->GetInfo();
				registerInfo.m_registerIndex = index;
				index ++;
				dAssert (index < D_CALLER_SAVE_REGISTER_COUNT);
			}
		}
	}

	for (iter.Begin(); iter; iter ++) {
		dDataFlowPoint& info = iter.GetNode()->GetInfo();
		if (info.m_generatedVariable.Size()) {
			const dString& variableA = info.m_generatedVariable;

            if (IsTempVariable(variableA)) {
				dTreeNode* const nodeA = Find(variableA);
				dAssert (nodeA);
				dRegisterInterferenceNode& inteferanceNodeA = nodeA->GetInfo();
				dDataFlowPoint::dVariableSet<dString>::Iterator aliveIter (info.m_liveOutputSet);

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
		dDataFlowPoint& info = iter.GetNode()->GetInfo();
		//dThreeAdressStmt& stmt = info.m_statement->GetInfo();
		dCILInstr* const instr = info.m_statement->GetInfo();
		if (instr->GetAsMove()) {
			dCILInstrMove* const move = instr->GetAsMove();
//			if ((stmt.m_instruction == dThreeAdressStmt::m_assigment) && (stmt.m_operator == dThreeAdressStmt::m_nothing) && (stmt.m_arg1.GetType().m_intrinsicType != dThreeAdressStmt::m_constInt) && (stmt.m_arg1.GetType().m_intrinsicType != dThreeAdressStmt::m_constFloat)) {
			const dString& variableA = move->GetArg0().m_label;
			const dString& variableB = move->GetArg1().m_label;
			dTreeNode* const nodeA = Find(variableA);
			dTreeNode* const nodeB = Find(variableB);

			if ((nodeA->GetInfo().m_registerIndex == -1) && (nodeB->GetInfo().m_registerIndex == -1)) {
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
					m_coalescedNodes.Append(dCoalescedNodePair (nodeB, nodeA));
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

//	m_flowGraph->m_savedRegistersMask = 0;
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

//	dTree<int, dString> callerSaveRegisterRemap;
	m_flowGraph->CalculateLiveInputLiveOutput ();
	for (dCIL::dListNode* node = m_flowGraph->m_basicBlocks.m_begin; node != m_flowGraph->m_basicBlocks.m_end; node = node->GetNext()) {
		//dThreeAdressStmt& stmt = stmtNode->GetInfo();
		dCILInstr* const instr = node->GetInfo();
		if (instr->GetAsCall()) {
			dCILInstrCall* const callInstr = instr->GetAsCall();
			dAssert (m_flowGraph->m_dataFlowGraph.Find (node));
			dDataFlowPoint& point = m_flowGraph->m_dataFlowGraph.Find (node)->GetInfo();
			dDataFlowPoint::dVariableSet<dString>::Iterator iter (point.m_liveOutputSet);
			for (iter.Begin(); iter; iter ++) {
				const dString& varName = iter.GetKey();
				//if (varName != stmt.m_arg0.m_label) {
				dTreeNode* const interferanceGraphNode = Find(varName);
				dRegisterInterferenceNode& interferanceGraphVariable = interferanceGraphNode->GetInfo();
				if (interferanceGraphVariable.m_registerIndex == -1) {
					interferanceGraphVariable.m_saveRegisterOnEntry = true;
					//callerSaveRegisterRemap.Insert (0, varName);
				}
			}
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
				if (variable.m_saveRegisterOnEntry) {
					index = D_CALLER_SAVE_REGISTER_COUNT;
					const int highMask = variable.m_saveRegisterOnEntry << D_CALLER_SAVE_REGISTER_COUNT;
					for (int mask = regMask; mask & highMask ; mask >>= 1) {
						index ++;
					}

					if (index > (D_INTEGER_REGISTER_COUNT - 1)) {
						dAssert (0);
						index = 0;
						for (int mask = regMask; mask & 1; mask >>= 1) {
							index ++;
						}
					}

				} else {
					index = 0;
					for (int mask = regMask; mask & 1; mask >>= 1) {
						index ++;
					}
				}

				if (index > (D_INTEGER_REGISTER_COUNT - 1)) {
					// spill
					return  D_INTEGER_REGISTER_COUNT;
					dAssert (0);
				}
				variable.m_registerIndex = index;

				if (variable.m_coalescedParent) {
					dRegisterInterferenceNode& parentVariable = variable.m_coalescedParent->GetInfo();
					parentVariable.m_registerIndex = variable.m_registerIndex;
				}
			}
		}
		//m_flowGraph->m_registersUsedMask |= (1 << variable.m_registerIndex);
		registersUsed = dMax (registersUsed, variable.m_registerIndex);
	}
#if 0
	// remap register
	int count = 0;
	int* registerCost = new int[registerOrder.GetCount()];
	dTreeNode** registerNode = new dTreeNode*[registerOrder.GetCount()];
	for (dList<dTreeNode*>::dListNode* node = registerOrder.GetFirst(); node; node = node->GetNext()) {
		dTreeNode* const varNode = node->GetInfo();
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		if (variable.m_saveRegisterOnEntry) {
			registerCost[count] = (variable.m_saveRegisterOnEntry << 24) + (variable.m_registerIndex << 1);
		} else {
			registerCost[count] = (variable.m_registerIndex << 1) + !variable.m_isPrecolored;
		}
		registerNode[count] = varNode;
		count ++;
	}

	for (int i = 1; i < count; i ++) {
		int cost = registerCost[i];
		dTreeNode* const varNode = registerNode[i];
		int j = i - 1;
		for (; (j >= 0) && (registerCost[j] > cost); j --) {
			registerCost[j + 1] = registerCost[j];
			registerNode[j + 1] = registerNode[j];
		}
		registerCost[j + 1] = cost;
		registerNode[j + 1] = varNode;
	}

	int saveOnEntryStart = 100000;
	for (int i = 0; i < count; i ++) {
		int cost = registerCost[i];
		if (cost >=  1 << 24) {
			saveOnEntryStart = i;
			break;
		}
	}

	int saveOnEntryIndex = D_CALLER_SAVE_REGISTER_COUNT;
	memset (registerCost, -1, count * sizeof (registerCost[0]));
	for (int i = 0; i < count; i ++) {
		dTreeNode* const varNode = registerNode[i];
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		int index = variable.m_registerIndex;
		if (registerCost[index] == -1) {
			if (variable.m_saveRegisterOnEntry) {
				registerCost[index] = saveOnEntryIndex;
				m_flowGraph->m_savedRegistersMask |= (1 << saveOnEntryIndex);
				saveOnEntryIndex ++;
			} else if (variable.m_isPrecolored) {
				registerCost[index] = variable.m_registerIndex;
			} else {
				registerCost[index] = variable.m_registerIndex;
			}
		}
		variable.m_registerIndex = registerCost[index];
	}
	
	delete[] registerCost;
	delete[] registerNode;
#endif
	return registersUsed + 1;
}


void dRegisterInterferenceGraph::SelectSpillVariableAndReWriteFunction()
{
/*
	dTree<dVariableSpillPriority,dString> spillPriority;
	dTree<dDataFlowGraph::dDataFlowPoint, dCIL::dListNode*>::Iterator stmtIter (m_flowGraph->m_dataFlowGraph);

	for (stmtIter.Begin(); stmtIter; stmtIter ++) {
		dDataFlowGraph::dDataFlowPoint& point = stmtIter.GetNode()->GetInfo();
//		dThreeAdressStmt& stmt = point.m_statement->GetInfo();	
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

			dThreeAdressStmt& stmt = node->GetInfo();	
//stmt.Trace();
			switch (stmt.m_instruction)
			{

				case dThreeAdressStmt::m_assigment:
				{
					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg0.m_label);
					if (priorityNode) {
						priorityNode->GetInfo().m_useCount --;
						priorityNode->GetInfo().m_loopUseCount ++;
					}

					if (stmt.m_arg1.m_type == dThreeAdressStmt::m_intVar) {
						dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg1.m_label);
						if (priorityNode) {
							priorityNode->GetInfo().m_useCount --;
							priorityNode->GetInfo().m_loopUseCount ++;
						}
					} else if (stmt.m_arg1.m_type == dThreeAdressStmt::m_floatVar) {
						dAssert (0);
					}

					if (stmt.m_operator != dThreeAdressStmt::m_nothing) {
						if (stmt.m_arg2.m_type == dThreeAdressStmt::m_intVar) {
							dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg2.m_label);
							if (priorityNode) {
								priorityNode->GetInfo().m_useCount --;
								priorityNode->GetInfo().m_loopUseCount ++;
							}
						} else if (stmt.m_arg2.m_type == dThreeAdressStmt::m_floatVar) {
							dAssert (0);
						}
					}
					break;
				}

				case dThreeAdressStmt::m_if:
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

				case dThreeAdressStmt::m_pop:
				case dThreeAdressStmt::m_push:
				case dThreeAdressStmt::m_storeBase:
				case dThreeAdressStmt::m_loadBase:
				{
					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode0 = spillPriority.Find(stmt.m_arg0.m_label);
					if (priorityNode0) {
						priorityNode0->GetInfo().m_useCount --;
						priorityNode0->GetInfo().m_loopUseCount ++;
					}
					break;
				}
                
   				case dThreeAdressStmt::m_load:
                case dThreeAdressStmt::m_store:
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

//				case dThreeAdressStmt::m_free:
//				case dThreeAdressStmt::m_alloc:
//				{
//					stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
//					stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
//					break;
//				}

				case dThreeAdressStmt::m_call:
				case dThreeAdressStmt::m_goto:
				case dThreeAdressStmt::m_label:
				case dThreeAdressStmt::m_nop:
				case dThreeAdressStmt::m_enter:
				case dThreeAdressStmt::m_leave:
				case dThreeAdressStmt::m_function:
				case dThreeAdressStmt::m_argument:
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
*/
}


void dRegisterInterferenceGraph::SaveSpillRegister(dCIL::dListNode* const node, dCILInstr::dArg& argument, const dString& spillVariable, const dString& spillMemory)
{
dAssert (0);
/*
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

		dThreeAdressStmt& stmt = newNode->GetInfo();
		stmt.m_instruction = dThreeAdressStmt::m_storeBase;
		stmt.m_operator = dThreeAdressStmt::m_nothing;
		stmt.m_arg0.m_label = newtemp;
		stmt.m_arg0.m_type = dThreeAdressStmt::m_intVar;
		stmt.m_arg2.m_label = spillMemory;
		stmt.m_arg2.m_type = dThreeAdressStmt::m_intVar;

//node->GetInfo().Trace();
//stmt.Trace();
	}
*/
}

void dRegisterInterferenceGraph::LoadSpillRegister(dCIL::dListNode* const node, dCILInstr::dArg& argument, const dString& spillVariable, const dString& spillMemory)
{
dAssert (0);
/*
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

		dThreeAdressStmt& stmt = newNode->GetInfo();
		stmt.m_instruction = dThreeAdressStmt::m_loadBase;
		stmt.m_operator = dThreeAdressStmt::m_nothing;
		stmt.m_arg0.m_label = newtemp;
		stmt.m_arg0.m_type = dThreeAdressStmt::m_intVar;
		stmt.m_arg2.m_label = spillMemory;
		stmt.m_arg2.m_type = dThreeAdressStmt::m_intVar;

//stmt.Trace();
//node->GetInfo().Trace();
	}
*/
}


void dRegisterInterferenceGraph::ReWriteFunctionSpillingVarible(const dString& spillVariable)
{
dAssert (0);
/*
//m_flowGraph->m_cil->Trace();

	dString spillLocal = IndexToLocal(m_spillCount);
	m_spillCount ++;

	dCIL::dListNode* nextNode;
	m_spillPenatryFactor += 50;
	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = nextNode) {
		nextNode = node->GetNext();
		dThreeAdressStmt& stmt = node->GetInfo();
//stmt.Trace();
		switch (stmt.m_instruction)
		{
            case dThreeAdressStmt::m_pop:
			case dThreeAdressStmt::m_loadBase:
			{
				SaveSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
				break;
			}

			case dThreeAdressStmt::m_assigment:
			{
				LoadSpillRegister (node, stmt.m_arg1, spillVariable, spillLocal);
				if (stmt.m_operator != dThreeAdressStmt::m_nothing) {
					LoadSpillRegister (node, stmt.m_arg2, spillVariable, spillLocal);
				}
				SaveSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
				break;
			}

			case dThreeAdressStmt::m_if:
			{
				LoadSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
				LoadSpillRegister (node, stmt.m_arg1, spillVariable, spillLocal);
				break;
			}

            case dThreeAdressStmt::m_load:
            {
                LoadSpillRegister (node, stmt.m_arg2, spillVariable, spillLocal);
                LoadSpillRegister (node, stmt.m_arg1, spillVariable, spillLocal);
                SaveSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
            }

			case dThreeAdressStmt::m_store:
			{
                LoadSpillRegister (node, stmt.m_arg2, spillVariable, spillLocal);
                LoadSpillRegister (node, stmt.m_arg1, spillVariable, spillLocal);
                LoadSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
				break;
			}

            case dThreeAdressStmt::m_push:
            case dThreeAdressStmt::m_storeBase:
            {
               LoadSpillRegister (node, stmt.m_arg0, spillVariable, spillLocal);
               break;
            }
		
//			case dThreeAdressStmt::m_alloc:
//			{
//				dAssert (0);
//				//point.m_killVariable = stmt.m_arg0.m_label;
//				//point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
//				break;
//			}
//
//			case dThreeAdressStmt::m_free:
//			{
//				dAssert (0);
//				//point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
//				break;
//			}
//
			
			case dThreeAdressStmt::m_nop:
			case dThreeAdressStmt::m_ret:
			case dThreeAdressStmt::m_goto:
			case dThreeAdressStmt::m_call:
			case dThreeAdressStmt::m_label:
			case dThreeAdressStmt::m_enter:
			case dThreeAdressStmt::m_leave:
			case dThreeAdressStmt::m_function:
			case dThreeAdressStmt::m_argument:
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
*/
}

void dRegisterInterferenceGraph::InsertEpilogAndProlog()
{
	int registerMask = 0;
	int lastReg = 0;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		dRegisterInterferenceNode& node = iter.GetNode()->GetInfo();
		lastReg = dMax(node.m_registerIndex, lastReg);
	}

	lastReg = lastReg - D_CALLER_SAVE_REGISTER_COUNT;
	if (lastReg >= 0) {
		registerMask = (1 << (lastReg + 1)) - 1;
	}

	int localMemory = 0;

	if (registerMask || localMemory) {

		dCILInstrFunction* const entryPoint = m_flowGraph->m_function->GetInfo()->GetAsFunction();
		dAssert(entryPoint);
		dCILInstrEnter* const enter = new dCILInstrEnter(*m_flowGraph->m_cil, entryPoint, registerMask, localMemory);
		
		for (dCIL::dListNode* node = m_flowGraph->m_function; node != m_flowGraph->m_basicBlocks.m_end; node = node->GetNext()) {
			dCILInstr* const instr = node->GetInfo();
			if (instr->GetAsReturn()) {
				new dCILInstrLeave(enter, instr->GetAsReturn());
			}
		}

//		m_flowGraph->m_cil->Trace();
	}
}