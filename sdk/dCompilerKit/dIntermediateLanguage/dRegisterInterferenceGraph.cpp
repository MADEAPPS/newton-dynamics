/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
#include "dCILInstrBranch.h"
#include "dBasicBlocksGraph.h"
#include "dCILInstrLoadStore.h"
#include "dCILInstrMiscellaneous.h"
#include "dRegisterInterferenceGraph.h"



bool dRegisterInterferenceGraph::IsSpilledVariable (const dString& name) const
{
	static int spillSize = strlen (D_SPILL_REGISTER_SYMBOL);
	return strncmp (name.GetStr(), D_SPILL_REGISTER_SYMBOL, spillSize) ? false : true;
}

void dRegisterInterferenceGraph::Build()
{
//m_graph->Trace();
	dLiveInLiveOutSolver liveInLiveOut(m_graph);
	for (dLiveInLiveOutSolver::dListNode* node = liveInLiveOut.GetFirst(); node; node = node->GetNext()) {
		dList<dCILInstr::dArg*> usedVariables;
		dLiveInLiveOut& point = node->GetInfo();
//point.m_instruction->Trace();		
		dCILInstr::dArg* const generated = point.m_instruction->GetGeneratedVariable();
		if (generated) {
//dTrace (("%s\n", generated->m_label.GetStr()));
			Insert(generated->m_label);
		}

		point.m_instruction->GetUsedVariables(usedVariables);
		for (dList<dCILInstr::dArg*>::dListNode* usedVarNode = usedVariables.GetFirst(); usedVarNode; usedVarNode = usedVarNode->GetNext()) {	
			const dString& variable = usedVarNode->GetInfo()->m_label;
//dTrace (("%s\n", variable.GetStr()));
			if (IsSpilledVariable(variable)) {
				dAssert (0);
				dTreeNode* interferanceGraphNode = Find(variable);
				if (!interferanceGraphNode) {
					interferanceGraphNode = Insert(variable);
					interferanceGraphNode->GetInfo().m_name = variable;
					dTrace (("%s\n", variable.GetStr()));
				}
			} else {
				Insert(variable);
			}
		}
	}

	// pre-color some special nodes
	//int intArgumentIndex = D_CALLER_SAVE_REGISTER_INDEX; 
	for (dLiveInLiveOutSolver::dListNode* node = liveInLiveOut.GetFirst(); node; node = node->GetNext()) {
		dLiveInLiveOut& point = node->GetInfo();
		dCILInstr* const instr = point.m_instruction;
		//instr->Trace();

		if (instr->GetAsArgument()) {
			dAssert(0);
/*
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
*/
		} else if (instr->GetAsReturn()) {
			dCILInstrReturn* const retInstr = instr->GetAsReturn();

			const dCILInstr::dArg& arg = retInstr->GetArg0();
			if (arg.GetType().m_isPointer || (arg.GetType().m_intrinsicType != dCILInstr::m_void)) {
				switch (arg.GetType().m_intrinsicType)
				{
					case dCILInstr::m_int:
					{
						dAssert (0);
						dTreeNode* const returnRegNode = Find(arg.m_label);
						dAssert(returnRegNode);
						dRegisterInterferenceNode& registerInfo = returnRegNode->GetInfo();
						registerInfo.m_registerIndex = D_RETURN_REGISTER_INDEX;
						registerInfo.m_isPrecolored = true;
						break;
					}

					case dCILInstr::m_luaType:
					{
						dTreeNode* const returnRegNode = Find(arg.m_label);
						dAssert(returnRegNode);
						dRegisterInterferenceNode& registerInfo = returnRegNode->GetInfo();
						registerInfo.m_registerIndex = D_RETURN_REGISTER_INDEX;
						registerInfo.m_isPrecolored = true;
						break;
					}
					default:
					{
						dAssert(0);
					}
				}
			}

		} else if (instr->GetAsCall()) {
			dAssert(0);
/*
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
*/
		}
	}

/*
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
*/

//m_graph->Trace();
	for (dLiveInLiveOutSolver::dListNode* node = liveInLiveOut.GetFirst(); node; node = node->GetNext()) {
		dLiveInLiveOut& point = node->GetInfo();
//point.m_instruction->Trace();
		const dVariableSet<dString>& liveIntSet = point.m_liveInputSet;

		dVariableSet<dString>::Iterator iterA(liveIntSet);
		for (iterA.Begin(); iterA; iterA++) {
			const dString& varA = iterA.GetKey();
			dTree<dRegisterInterferenceNode, dString>::dTreeNode* const varAnodeNode = Find(varA);
			dRegisterInterferenceNode& varAnode = varAnodeNode->GetInfo();
			dVariableSet<dString>::Iterator iterB(iterA);
			for (iterB++; iterB; iterB++) {
				const dString& varB = iterB.GetKey();
				if (!varAnode.FindEdge(varB)) {
					dTree<dRegisterInterferenceNode, dString>::dTreeNode* const varBnodeNode = Find(varB);
					dRegisterInterferenceNode& varBnode = varBnodeNode->GetInfo();
					dRegisterInterferenceNodeEdge edgeAB(varBnodeNode);
					dRegisterInterferenceNodeEdge edgeBA(varAnodeNode);
					varAnode.m_interferanceEdge.Append(edgeAB);
					varBnode.m_interferanceEdge.Append(edgeBA);
					//dTrace(("%s  %s\n", varA.GetStr(), varB.GetStr()));
				}
			}
		}
	}

//m_graph->Trace();
	for (dLiveInLiveOutSolver::dListNode* node = liveInLiveOut.GetFirst(); node; node = node->GetNext()) {
		dLiveInLiveOut& point = node->GetInfo();
		dCILInstr* const instr = point.m_instruction;
		if (instr->GetAsMove()) {
			dCILInstrMove* const move = instr->GetAsMove();
			dList<dCILInstr::dArg*> variablesList;
			move->GetUsedVariables(variablesList);
//move->Trace();
			if (variablesList.GetCount()) {
				//if ((stmt.m_instruction == dThreeAdressStmt::m_assigment) && (stmt.m_operator == dThreeAdressStmt::m_nothing) && (stmt.m_arg1.GetType().m_intrinsicType != dThreeAdressStmt::m_constInt) && (stmt.m_arg1.GetType().m_intrinsicType != dThreeAdressStmt::m_constFloat)) {
				const dString& variableA = move->GetArg0().m_label;
				const dString& variableB = move->GetArg1().m_label;
				dTreeNode* const nodeA = Find(variableA);
				dTreeNode* const nodeB = Find(variableB);
				nodeA;
				nodeB;
			}
		}
	}
}


dRegisterInterferenceNodeEdge* dRegisterInterferenceNode::FindEdge(const dString& var)
{
	for (dList<dRegisterInterferenceNodeEdge>::dListNode* node = m_interferanceEdge.GetFirst(); node; node = node->GetNext()) {
		dRegisterInterferenceNodeEdge& info = node->GetInfo();
		const dString& edgeVar = info.m_incidentNode->GetKey();
		if (edgeVar == var) {
			return &info;
		}
	}
	return NULL;
}


dRegisterInterferenceGraph::dRegisterInterferenceGraph (dBasicBlocksGraph* const graph, int registerCount)
	:dTree<dRegisterInterferenceNode, dString>()
	,m_graph(graph)
{
//	m_flowGraph->ApplySemanticInstructionReordering();
	Build();
	while (ColorGraph () > registerCount) {
		// we have a spill, find a good spill node and try graph coloring again
		dAssert (0);
	}

	AllocateRegisters();
/*
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
*/
}



dRegisterInterferenceGraph::dTreeNode* dRegisterInterferenceGraph::GetBestNode(int& edgeCount)
{
	int minCount = 0x7fffff;
	dTreeNode* bestNode = NULL;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter++) {
		dTreeNode* const node = iter.GetNode();
		dRegisterInterferenceNode& info = node->GetInfo();
		if (!info.m_inSet) {
			int count = 0;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = info.m_interferanceEdge.GetFirst(); edgeNode && (count < minCount); edgeNode = edgeNode->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				if (!edge.m_mark) {
					count++;
				}
			}
			if (count < minCount) {
				minCount = count;
				bestNode = node;
			}
		}
	}

	edgeCount = minCount;
	dRegisterInterferenceNode& info = bestNode->GetInfo();
	info.m_inSet = true;
	return bestNode;
}


int dRegisterInterferenceGraph::ColorGraph()
{
#if 0
	dList<dTreeNode*> registerOrder;
	for (int i = 0; i < GetCount(); i++) {
		int edgeCount; 
		dTreeNode* const bestNode = GetBestNode(edgeCount);
dTrace (("%s\n", bestNode->GetKey().GetStr()));
		registerOrder.Addtop(bestNode);
		dRegisterInterferenceNode& node = bestNode->GetInfo();
		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = node.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			edge.m_mark = true;
			dRegisterInterferenceNode* const twinNode = &edge.m_incidentNode->GetInfo();
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* twinPtr = twinNode->m_interferanceEdge.GetFirst(); twinPtr; twinPtr = twinPtr->GetNext()) {
				dRegisterInterferenceNodeEdge& twinEdge = twinPtr->GetInfo();
				if (twinEdge.m_incidentNode == bestNode) {
					twinEdge.m_mark = true;
					break;
				}
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
				const dRegisterInterferenceNode& otherVariable = edge.m_incidentNode->GetInfo();
				if (otherVariable.m_registerIndex != -1) {
					regMask |= (1 << otherVariable.m_registerIndex);
				}
			}

			int index = 0;
			if (variable.m_saveRegisterOnEntry) {
				dAssert(0);
				//index = D_CALLER_SAVE_REGISTER_COUNT;
				//const int highMask = variable.m_saveRegisterOnEntry << D_CALLER_SAVE_REGISTER_COUNT;
				//for (int mask = regMask; mask & highMask; mask >>= 1) {
				//	index++;
				//}
				//if (index > (D_INTEGER_REGISTER_COUNT - 1)) {
				//	dAssert(0);
				//	index = 0;
				//	for (int mask = regMask; mask & 1; mask >>= 1) {
				//		index++;
				//	}
				//}
			} else {
				for (int mask = regMask; mask & 1; mask >>= 1) {
					index++;
				}
			}

			if (index > (D_INTEGER_REGISTER_COUNT - 1)) {
				// spill
				dAssert(0);
				return  D_INTEGER_REGISTER_COUNT;
				
			}
			variable.m_registerIndex = index;
		}
		registersUsed = dMax(registersUsed, variable.m_registerIndex);
	}

	// remap register
	int count = 0;
	int registerCost[1024];
	dTreeNode* registerNode[1024];

	m_graph->m_savedRegistersMask = 0;
	dAssert(registerOrder.GetCount() < sizeof(registerCost) / sizeof(registerCost[0]));
	for (dList<dTreeNode*>::dListNode* node = registerOrder.GetFirst(); node; node = node->GetNext()) {
		dTreeNode* const varNode = node->GetInfo();
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		if (variable.m_saveRegisterOnEntry) {
			registerCost[count] = (variable.m_saveRegisterOnEntry << 24) + (variable.m_registerIndex << 1);
		} else {
			registerCost[count] = (variable.m_registerIndex << 1) + !variable.m_isPrecolored;
		}
		registerNode[count] = varNode;
		count++;
	}

	for (int i = 1; i < count; i++) {
		int cost = registerCost[i];
		dTreeNode* const varNode = registerNode[i];
		int j = i - 1;
		for (; (j >= 0) && (registerCost[j] > cost); j--) {
			registerCost[j + 1] = registerCost[j];
			registerNode[j + 1] = registerNode[j];
		}
		registerCost[j + 1] = cost;
		registerNode[j + 1] = varNode;
	}

	int saveOnEntryStart = 100000;
	for (int i = 0; i < count; i++) {
		int cost = registerCost[i];
		if (cost >= 1 << 24) {
			saveOnEntryStart = i;
			break;
		}
	}
#else

	// my algorithm for register allocation from interference graph, seems better than Chaitin method
	// at least is much faster and yidl teh same result.
	Iterator iter(*this);
	int registersUsed = -1;
	for (iter.Begin(); iter; iter++) {
		dTreeNode* const node = iter.GetNode();
		dRegisterInterferenceNode& variable = node->GetInfo();
		if (variable.m_registerIndex == -1) {
			int regMask = 0;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = variable.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
				const dRegisterInterferenceNode& otherVariable = edge.m_incidentNode->GetInfo();
				if (otherVariable.m_registerIndex != -1) {
					regMask |= (1 << otherVariable.m_registerIndex);
				}
			}

			int index = 0;
			if (variable.m_saveRegisterOnEntry) {
				dAssert(0);
				//index = D_CALLER_SAVE_REGISTER_COUNT;
				//const int highMask = variable.m_saveRegisterOnEntry << D_CALLER_SAVE_REGISTER_COUNT;
				//for (int mask = regMask; mask & highMask; mask >>= 1) {
				//	index++;
				//}
				//if (index > (D_INTEGER_REGISTER_COUNT - 1)) {
				//	dAssert(0);
				//	index = 0;
				//	for (int mask = regMask; mask & 1; mask >>= 1) {
				//		index++;
				//	}
				//}
			} else {
				for (int mask = regMask; mask & 1; mask >>= 1) {
					index++;
				}
			}

			if (index > (D_INTEGER_REGISTER_COUNT - 1)) {
				// spill
				dAssert(0);
				return  D_INTEGER_REGISTER_COUNT;
			}
			variable.m_registerIndex = index;
		}
		registersUsed = dMax(registersUsed, variable.m_registerIndex);
	}

	int count = 0;
	int registerCost[1024];
	dTreeNode* registerNode[1024];

	m_graph->m_savedRegistersMask = 0;
	dAssert(GetCount() < sizeof(registerCost) / sizeof(registerCost[0]));
	for (iter.Begin(); iter; iter++) {
		dTreeNode* const varNode = iter.GetNode();
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		if (variable.m_saveRegisterOnEntry) {
			registerCost[count] = (variable.m_saveRegisterOnEntry << 24) + (variable.m_registerIndex << 1);
		} else {
			registerCost[count] = (variable.m_registerIndex << 1) + !variable.m_isPrecolored;
		}
		registerNode[count] = varNode;
		count++;
	}
#endif

	int saveOnEntryIndex = D_CALLER_SAVE_REGISTER_COUNT;
	memset(registerCost, -1, count * sizeof(registerCost[0]));
	for (int i = 0; i < count; i++) {
		dTreeNode* const varNode = registerNode[i];
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		int index = variable.m_registerIndex;
		if (registerCost[index] == -1) {
			if (variable.m_saveRegisterOnEntry) {
				registerCost[index] = saveOnEntryIndex;
				m_graph->m_savedRegistersMask |= (1 << saveOnEntryIndex);
				saveOnEntryIndex++;
			}
			else if (variable.m_isPrecolored) {
				registerCost[index] = variable.m_registerIndex;
			}
			else {
				registerCost[index] = variable.m_registerIndex;
			}
		}
		variable.m_registerIndex = registerCost[index];
	}
	return registersUsed + 1;
}


void dRegisterInterferenceGraph::AllocateRegisters()
{
	for (dList<dBasicBlock>::dListNode* blockNode = m_graph->GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		for (dCIL::dListNode* node = block.m_end; node != block.m_begin; node = node->GetPrev()) {
			dCILInstr* const instr = node->GetInfo();
//instr->Trace();
			instr->AssignRegisterName(*this);
			//instr->Trace();
		}
	}
}


int dRegisterInterferenceGraph::GetRegisterIndex(const dString& varName) const
{
	int index = -1;
	//	if (varName[0] != '_') {
	dTreeNode* const node = Find(varName);
	dRegisterInterferenceNode& var = node->GetInfo();
	index = var.m_registerIndex;
	//	}
	return index;
}


dString dRegisterInterferenceGraph::GetRegisterName(const dString& varName) const
{
	int index = GetRegisterIndex(varName);
	if (index == -1) {
		dAssert(0);
		return varName;
	} else {
		return IndexToRegister(index);
	}
}
